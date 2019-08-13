import tensorflow.python as tf
from tensorflow.python import keras
from pathlib import Path
from tools.utils import Helper, INFO, ERROR, NOTE, tf_xywh_to_all
from tools.alignutils import YOLOAlignHelper, tf_grid_to_all
from models.yolonet import *
from termcolor import colored
from PIL import Image, ImageFont, ImageDraw
import argparse
import sys
import numpy as np
from register import dict2obj, network_register, optimizer_register
from yaml import safe_load

tf.enable_eager_execution()
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
keras.backend.set_session(sess)
keras.backend.set_learning_phase(0)


def correct_box(box_xy: tf.Tensor, box_wh: tf.Tensor, input_shape: list, image_hw: list) -> tf.Tensor:
    """rescae predict box to orginal image scale

    Parameters
    ----------
    box_xy : tf.Tensor
        box xy
    box_wh : tf.Tensor
        box wh
    input_shape : list
        input shape
    image_hw : list
        image shape

    Returns
    -------
    tf.Tensor
        new boxes
    """
    box_yx = box_xy[..., ::-1]
    box_hw = box_wh[..., ::-1]
    input_shape = tf.cast(input_shape, tf.float32)
    image_hw = tf.cast(image_hw, tf.float32)
    new_shape = tf.round(image_hw * tf.reduce_min(input_shape / image_hw))
    offset = (input_shape - new_shape) / 2. / input_shape
    scale = input_shape / new_shape
    box_yx = (box_yx - offset) * scale
    box_hw *= scale

    box_mins = box_yx - (box_hw / 2.)
    box_maxes = box_yx + (box_hw / 2.)
    boxes = tf.concat([
        box_mins[..., 0:1],  # y_min
        box_mins[..., 1:2],  # x_min
        box_maxes[..., 0:1],  # y_max
        box_maxes[..., 1:2]  # x_max
    ], axis=-1)

    # Scale boxes back to original image shape.
    boxes *= tf.concat([image_hw, image_hw], axis=-1)
    return boxes


def correct_algin_box(box_xy: tf.Tensor, box_wh: tf.Tensor, landmark: tf.Tensor, input_shape: list, image_hw: list) -> tf.Tensor:
    """rescae predict box to orginal image scale

    Parameters
    ----------
    box_xy : tf.Tensor
        box xy
    box_wh : tf.Tensor
        box wh
    input_shape : list
        input shape
    image_hw : list
        image shape

    Returns
    -------
    tf.Tensor
        new boxes
    """
    box_yx = box_xy[..., ::-1]
    box_hw = box_wh[..., ::-1]
    input_shape = tf.cast(input_shape, tf.float32)
    image_hw = tf.cast(image_hw, tf.float32)
    new_shape = tf.round(image_hw * tf.reduce_min(input_shape / image_hw))
    offset = (input_shape - new_shape) / 2. / input_shape
    scale = input_shape / new_shape
    box_yx = (box_yx - offset) * scale
    box_hw *= scale

    box_mins = box_yx - (box_hw / 2.)
    box_maxes = box_yx + (box_hw / 2.)
    boxes = tf.concat([
        box_mins[..., 0:1],  # y_min
        box_mins[..., 1:2],  # x_min
        box_maxes[..., 0:1],  # y_max
        box_maxes[..., 1:2]  # x_max
    ], axis=-1)

    # Scale boxes back to original image shape.
    boxes *= tf.concat([image_hw, image_hw], axis=-1)
    return boxes


def parser_yolo_output(orig_img: np.ndarray, y_pred: list, input_hw: tuple,
                       image_hw: tuple, class_num: int, obj_thresh: float,
                       iou_thresh: float, h: Helper):
    """ box list """
    _yxyx_box = []
    _yxyx_box_scores = []
    """ preprocess label """
    for l, pred_label in enumerate(y_pred):
        """ split the label """
        pred_xy = pred_label[..., 0:2]
        pred_wh = pred_label[..., 2:4]
        pred_confidence = pred_label[..., 4:5]
        pred_cls = pred_label[..., 5:]
        # box_scores = obj_score * class_score
        box_scores = tf.sigmoid(pred_cls) * tf.sigmoid(pred_confidence)
        # obj_mask = pred_confidence_score[..., 0] > model.obj_thresh
        """ reshape box  """
        # NOTE tf_xywh_to_all will auto use sigmoid function
        pred_xy_A, pred_wh_A = tf_xywh_to_all(pred_xy, pred_wh, l, h)
        boxes = correct_box(pred_xy_A, pred_wh_A, input_hw, image_hw)
        boxes = tf.reshape(boxes, (-1, 4))
        box_scores = tf.reshape(box_scores, (-1, class_num))
        """ append box and scores to global list """
        _yxyx_box.append(boxes)
        _yxyx_box_scores.append(box_scores)

    yxyx_box = tf.concat(_yxyx_box, axis=0)
    yxyx_box_scores = tf.concat(_yxyx_box_scores, axis=0)

    mask = yxyx_box_scores >= obj_thresh

    """ do nms for every classes"""
    _boxes = []
    _scores = []
    _classes = []
    for c in range(class_num):
        class_boxes = tf.boolean_mask(yxyx_box, mask[:, c])
        class_box_scores = tf.boolean_mask(yxyx_box_scores[:, c], mask[:, c])
        select = tf.image.non_max_suppression(
            class_boxes, scores=class_box_scores, max_output_size=30, iou_threshold=iou_thresh)
        class_boxes = tf.gather(class_boxes, select)
        class_box_scores = tf.gather(class_box_scores, select)
        _boxes.append(class_boxes)
        _scores.append(class_box_scores)
        _classes.append(tf.ones_like(class_box_scores) * c)

    boxes = tf.concat(_boxes, axis=0)
    classes = tf.concat(_classes, axis=0)
    scores = tf.concat(_scores, axis=0)

    """ draw box  """
    font = ImageFont.truetype(font='asset/FiraMono-Medium.otf',
                              size=tf.cast(tf.floor(3e-2 * image_hw[0] + 0.5), tf.int32).numpy())

    thickness = (image_hw[0] + image_hw[1]) // 300

    """ show result """
    if len(classes) > 0:
        pil_img = Image.fromarray(orig_img)
        print(f'[top\tleft\tbottom\tright\tscore\tclass]')
        for i, c in enumerate(classes):
            box = boxes[i]
            score = scores[i]
            label = '{:2d} {:.2f}'.format(int(c.numpy()), score.numpy())
            draw = ImageDraw.Draw(pil_img)
            label_size = draw.textsize(label, font)
            top, left, bottom, right = box
            print(f'[{top:.1f}\t{left:.1f}\t{bottom:.1f}\t{right:.1f}\t{score:.2f}\t{int(c):2d}]')
            top = max(0, tf.cast(tf.floor(top + 0.5), tf.int32))
            left = max(0, tf.cast(tf.floor(left + 0.5), tf.int32))
            bottom = min(image_hw[0], tf.cast(tf.floor(bottom + 0.5), tf.int32))
            right = min(image_hw[1], tf.cast(tf.floor(right + 0.5), tf.int32))

            if top - image_hw[0] >= 0:
                text_origin = tf.convert_to_tensor([left, top - label_size[1]])
            else:
                text_origin = tf.convert_to_tensor([left, top + 1])

            for j in range(thickness):
                draw.rectangle(
                    [left + j, top + j, right - j, bottom - j],
                    outline=h.colormap[c])
            draw.rectangle(
                [tuple(text_origin), tuple(text_origin + label_size)],
                fill=h.colormap[c])
            draw.text(text_origin, label, fill=(0, 0, 0), font=font)
            del draw
        pil_img.show()
    else:
        print(NOTE, ' no boxes detected')


def parser_yoloalgin_output(orig_img: np.ndarray, y_pred: list, input_hw: tuple,
                            image_hw: tuple, class_num: int, landmark_num: int,
                            obj_thresh: float, iou_thresh: float, h: YOLOAlignHelper):
    """ box list """
    _yxyx_box = []
    _yxyx_box_scores = []
    """ preprocess label """
    for l, pred_label in enumerate(y_pred):
        """ split the label """
        pred_xy = pred_label[..., 0:2]
        pred_wh = pred_label[..., 2:4]
        pred_confidence = pred_label[..., 4:5]
        pred_landmark = tf.reshape(pred_label[..., 5:5 + landmark_num * 2],
                                   pred_label.shape.as_list()[:-1] + [landmark_num, 2])
        # TODO 把解析函数搞定
        pred_cls = pred_label[..., 5 + landmark_num * 2:]
        # box_scores = obj_score * class_score
        box_scores = tf.sigmoid(pred_cls) * tf.sigmoid(pred_confidence)
        # obj_mask = pred_confidence_score[..., 0] > model.obj_thresh
        """ reshape box  """
        # NOTE tf_xywh_to_all will auto use sigmoid function
        pred_xy_A, pred_wh_A, pred_landmark_A = tf_grid_to_all(pred_xy, pred_wh, pred_landmark, l, h)
        boxes = correct_box(pred_xy_A, pred_wh_A, pred_landmark_A, input_hw, image_hw)
        boxes = tf.reshape(boxes, (-1, 4))
        box_scores = tf.reshape(box_scores, (-1, class_num))
        """ append box and scores to global list """
        _yxyx_box.append(boxes)
        _yxyx_box_scores.append(box_scores)

    yxyx_box = tf.concat(_yxyx_box, axis=0)
    yxyx_box_scores = tf.concat(_yxyx_box_scores, axis=0)

    mask = yxyx_box_scores >= obj_thresh

    """ do nms for every classes"""
    _boxes = []
    _scores = []
    _classes = []
    for c in range(class_num):
        class_boxes = tf.boolean_mask(yxyx_box, mask[:, c])
        class_box_scores = tf.boolean_mask(yxyx_box_scores[:, c], mask[:, c])
        select = tf.image.non_max_suppression(
            class_boxes, scores=class_box_scores, max_output_size=30, iou_threshold=iou_thresh)
        class_boxes = tf.gather(class_boxes, select)
        class_box_scores = tf.gather(class_box_scores, select)
        _boxes.append(class_boxes)
        _scores.append(class_box_scores)
        _classes.append(tf.ones_like(class_box_scores) * c)

    boxes = tf.concat(_boxes, axis=0)
    classes = tf.concat(_classes, axis=0)
    scores = tf.concat(_scores, axis=0)

    """ draw box  """
    font = ImageFont.truetype(font='asset/FiraMono-Medium.otf',
                              size=tf.cast(tf.floor(3e-2 * image_hw[0] + 0.5), tf.int32).numpy())

    thickness = (image_hw[0] + image_hw[1]) // 300

    """ show result """
    if len(classes) > 0:
        pil_img = Image.fromarray(orig_img)
        print(f'[top\tleft\tbottom\tright\tscore\tclass]')
        for i, c in enumerate(classes):
            box = boxes[i]
            score = scores[i]
            label = '{:2d} {:.2f}'.format(int(c.numpy()), score.numpy())
            draw = ImageDraw.Draw(pil_img)
            label_size = draw.textsize(label, font)
            top, left, bottom, right = box
            print(f'[{top:.1f}\t{left:.1f}\t{bottom:.1f}\t{right:.1f}\t{score:.2f}\t{int(c):2d}]')
            top = max(0, tf.cast(tf.floor(top + 0.5), tf.int32))
            left = max(0, tf.cast(tf.floor(left + 0.5), tf.int32))
            bottom = min(image_hw[0], tf.cast(tf.floor(bottom + 0.5), tf.int32))
            right = min(image_hw[1], tf.cast(tf.floor(right + 0.5), tf.int32))

            if top - image_hw[0] >= 0:
                text_origin = tf.convert_to_tensor([left, top - label_size[1]])
            else:
                text_origin = tf.convert_to_tensor([left, top + 1])

            for j in range(thickness):
                draw.rectangle(
                    [left + j, top + j, right - j, bottom - j],
                    outline=h.colormap[c])
            draw.rectangle(
                [tuple(text_origin), tuple(text_origin + label_size)],
                fill=h.colormap[c])
            draw.text(text_origin, label, fill=(0, 0, 0), font=font)
            del draw
        pil_img.show()
    else:
        print(NOTE, ' no boxes detected')


def main(ckpt_path: Path, model, train, test_image: Path):
    if model.name == 'yolo':
        h = Helper(None, model.class_num, f'data/{train.dataset}_anchor.npy',
                   model.input_hw, np.reshape(np.array(model.output_hw), (-1, 2)), train.vail_split)
        h.set_dataset(train.batch_size, train.rand_seed, is_training=train.augmenter)
    elif model.name == 'yoloalgin':
        h = YOLOAlignHelper(None, model.class_num, f'data/{train.dataset}_anchor.npy',
                            model.landmark_num, model.input_hw, np.reshape(np.array(model.output_hw), (-1, 2)),
                            train.vail_split)
        h.set_dataset(train.batch_size, train.rand_seed, is_training=train.augmenter)

    network = network_register[model.network]  # type :yolo_mobilev2
    if model.name == 'yolo':
        saved_model, trainable_model = network([model.input_hw[0], model.input_hw[1], 3],
                                               len(h.anchors[0]), model.class_num,
                                               alpha=model.depth_multiplier)
    elif model.name == 'yoloalgin':
        saved_model, trainable_model = network([model.input_hw[0], model.input_hw[1], 3],
                                               len(h.anchors[0]), model.class_num, model.landmark_num,
                                               alpha=model.depth_multiplier)

    trainable_model.load_weights(str(ckpt_path))
    print(INFO, f' Load CKPT {str(ckpt_path)}')
    """ load images """
    orig_img = h.read_img(str(test_image))
    image_hw = orig_img.shape[0:2]
    img, _ = h.process_img(orig_img, true_box=None, is_training=False, is_resize=True)
    img = tf.expand_dims(img, 0)
    """ get output """
    y_pred = trainable_model.predict(img)

    """ parser output """


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('pre_ckpt', type=str, help='pre-train weights path')
    parser.add_argument('--config', type=str, help='config file path, default in same folder with `pre_ckpt`', default=None)
    parser.add_argument('test_image', type=str, help='test image path')
    args = parser.parse_args(sys.argv[1:])

    pre_ckpt = Path(args.pre_ckpt)

    if args.config == None:
        config_path = list(pre_ckpt.parent.glob('*.yml'))[0]  # type: Path
    else:
        config_path = Path(args.config)

    with config_path.open('r') as f:
        cfg = safe_load(f)

    ArgMap = dict2obj(cfg)
    main(Path(args.pre_ckpt), ArgMap.model, ArgMap.train, Path(args.test_image))
