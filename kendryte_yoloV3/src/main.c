#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "board_config.h"
#include "dvp.h"
#include "fpioa.h"
#include "lcd.h"
#include "w25qxx.h"
#if OV5640
#include "ov5640.h"
#endif
#if OV2640
#include "ov2640.h"
#endif
#include "kpu.h"
#include "nt35310.h"
#include "plic.h"
#include "region_layer.h"
#include "sysctl.h"
#include "uarths.h"
#include "utils.h"
#define INCBIN_STYLE INCBIN_STYLE_SNAKE
#define INCBIN_PREFIX
#include "aiimg.h"
#include "incbin.h"
#include "iomem.h"

#define PLL0_OUTPUT_FREQ 800000000UL
#define PLL1_OUTPUT_FREQ 400000000UL

#define CLASS_NUMBER 20

#define LOAD_KMODEL_FROM_FLASH 0

#if LOAD_KMODEL_FROM_FLASH
#define KMODEL_SIZE (1351592)
uint8_t *model_data;
#else
INCBIN(model, "yolo3.kmodel");
#endif

kpu_model_context_t task;
static region_layer_t detect_rl, detect_rl0, detect_rl1;

volatile uint8_t g_ai_done_flag;

static void ai_done(void *ctx) { g_ai_done_flag = 1; }

uint32_t *g_lcd_gram0;
uint32_t *g_lcd_gram1;
uint8_t *g_ai_buf;

#define INPUT_WIDTH 320
#define INPUT_HEIGHT 240

#define L0_W 10
#define L0_H 7
#define L1_W 20
#define L1_H 14

// #define ANCHOR_NUM 5
// float g_anchor[ANCHOR_NUM * 2] = {1.08,  1.19, 3.42, 4.41,  6.63,
//                                   11.38, 9.42, 5.11, 16.62, 10.52};

#define ANCHOR_NUM 3
static float layer0_anchor[ANCHOR_NUM * 2] = {
    0.76120044 * L0_W, 0.57155991 * L0_H, 0.6923348 * L0_W,
    0.88535553 * L0_H, 0.47163042 * L0_W, 0.34163313 * L0_H,
};

static float layer1_anchor[ANCHOR_NUM * 2] = {
    0.33340788 * L1_W, 0.70065861 * L1_H, 0.18124964 * L1_W,
    0.38986752 * L1_H, 0.08497349 * L1_W, 0.1527057 * L1_H,
};

// static float layer0_anchor[ANCHOR_NUM * 2] = {
//     0.79283702 * L0_W, 0.7783739 * L0_H,  0.52263796 * L0_W,
//     0.38029872 * L0_H, 0.42174624 * L0_W, 0.77618033 * L0_H,
// };

// static float layer1_anchor[ANCHOR_NUM * 2] = {
//     0.22667582 * L1_W, 0.58617029 * L1_H, 0.17564948 * L1_W,
//     0.28898656 * L1_H, 0.07261302 * L1_W, 0.14261315 * L1_H,
// };

volatile uint8_t g_dvp_finish_flag = 0;
volatile uint8_t g_ram_mux = 0;

static int on_irq_dvp(void *ctx) {
  if (dvp_get_interrupt(DVP_STS_FRAME_FINISH)) {
    /* switch gram */
    dvp_set_display_addr(g_ram_mux ? (uint32_t)g_lcd_gram0
                                   : (uint32_t)g_lcd_gram1);

    dvp_clear_interrupt(DVP_STS_FRAME_FINISH);
    g_dvp_finish_flag = 1;
  } else {
    if (g_dvp_finish_flag == 0) dvp_start_convert();
    dvp_clear_interrupt(DVP_STS_FRAME_START);
  }

  return 0;
}

#if BOARD_LICHEEDAN
static void io_mux_init(void) {
  /* Init DVP IO map and function settings */
  fpioa_set_function(42, FUNC_CMOS_RST);
  fpioa_set_function(44, FUNC_CMOS_PWDN);
  fpioa_set_function(46, FUNC_CMOS_XCLK);
  fpioa_set_function(43, FUNC_CMOS_VSYNC);
  fpioa_set_function(45, FUNC_CMOS_HREF);
  fpioa_set_function(47, FUNC_CMOS_PCLK);
  fpioa_set_function(41, FUNC_SCCB_SCLK);
  fpioa_set_function(40, FUNC_SCCB_SDA);

  /* Init SPI IO map and function settings */
  fpioa_set_function(38, FUNC_GPIOHS0 + DCX_GPIONUM);
  fpioa_set_function(36, FUNC_SPI0_SS3);
  fpioa_set_function(39, FUNC_SPI0_SCLK);
  fpioa_set_function(37, FUNC_GPIOHS0 + RST_GPIONUM);

  sysctl_set_spi0_dvp_data(1);
}

static void io_set_power(void) {
  /* Set dvp and spi pin to 1.8V */
  sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
  sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
}

#else
static void io_mux_init(void) {
  /* Init DVP IO map and function settings */
  fpioa_set_function(11, FUNC_CMOS_RST);
  fpioa_set_function(13, FUNC_CMOS_PWDN);
  fpioa_set_function(14, FUNC_CMOS_XCLK);
  fpioa_set_function(12, FUNC_CMOS_VSYNC);
  fpioa_set_function(17, FUNC_CMOS_HREF);
  fpioa_set_function(15, FUNC_CMOS_PCLK);
  fpioa_set_function(10, FUNC_SCCB_SCLK);
  fpioa_set_function(9, FUNC_SCCB_SDA);

  /* Init SPI IO map and function settings */
  fpioa_set_function(8, FUNC_GPIOHS0 + DCX_GPIONUM);
  fpioa_set_function(6, FUNC_SPI0_SS3);
  fpioa_set_function(7, FUNC_SPI0_SCLK);

  sysctl_set_spi0_dvp_data(1);
}

static void io_set_power(void) {
  /* Set dvp and spi pin to 1.8V */
  sysctl_set_power_mode(SYSCTL_POWER_BANK1, SYSCTL_POWER_V18);
  sysctl_set_power_mode(SYSCTL_POWER_BANK2, SYSCTL_POWER_V18);
}
#endif

#if (CLASS_NUMBER > 1)
typedef struct {
  char *str;
  uint16_t color;
  uint16_t height;
  uint16_t width;
  uint32_t *ptr;
} class_lable_t;

class_lable_t class_lable[CLASS_NUMBER] = {
    {"aeroplane", GREEN},   {"bicycle", GREEN},     {"bird", GREEN},
    {"boat", GREEN},        {"bottle", 0xF81F},     {"bus", GREEN},
    {"car", GREEN},         {"cat", GREEN},         {"chair", 0xFD20},
    {"cow", GREEN},         {"diningtable", GREEN}, {"dog", GREEN},
    {"horse", GREEN},       {"motorbike", GREEN},   {"person", 0xF800},
    {"pottedplant", GREEN}, {"sheep", GREEN},       {"sofa", GREEN},
    {"train", GREEN},       {"tvmonitor", 0xF9B6}};

static uint32_t lable_string_draw_ram[115 * 16 * 8 / 2];
#endif

static void lable_init(void) {
#if (CLASS_NUMBER > 1)
  uint8_t index;

  class_lable[0].height = 16;
  class_lable[0].width = 8 * strlen(class_lable[0].str);
  class_lable[0].ptr = lable_string_draw_ram;
  lcd_ram_draw_string(class_lable[0].str, class_lable[0].ptr, BLACK,
                      class_lable[0].color);
  for (index = 1; index < CLASS_NUMBER; index++) {
    class_lable[index].height = 16;
    class_lable[index].width = 8 * strlen(class_lable[index].str);
    class_lable[index].ptr =
        class_lable[index - 1].ptr +
        class_lable[index - 1].height * class_lable[index - 1].width / 2;
    lcd_ram_draw_string(class_lable[index].str, class_lable[index].ptr, BLACK,
                        class_lable[index].color);
  }
#endif
}

static void drawboxes(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2,
                      uint32_t class, float prob) {
  if (x1 >= INPUT_WIDTH) x1 = INPUT_WIDTH - 1;
  if (x2 >= INPUT_WIDTH) x2 = INPUT_WIDTH - 1;
  if (y1 >= INPUT_HEIGHT) y1 = INPUT_HEIGHT - 1;
  if (y2 >= INPUT_HEIGHT) y2 = INPUT_HEIGHT - 1;

#if (CLASS_NUMBER > 1)
  lcd_draw_rectangle(x1, y1, x2, y2, 2, class_lable[class].color);
  lcd_draw_picture(x1 + 1, y1 + 1, class_lable[class].width,
                   class_lable[class].height, class_lable[class].ptr);
#else
  lcd_draw_rectangle(x1, y1, x2, y2, 2, RED);
#endif
}

int main(void) {
  /* Set CPU and dvp clk */
  sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_OUTPUT_FREQ);
  sysctl_pll_set_freq(SYSCTL_PLL1, PLL1_OUTPUT_FREQ);
  sysctl_clock_enable(SYSCTL_CLOCK_AI);

  uarths_init();
  io_mux_init();
  io_set_power();
  plic_init();

  /* flash init */
  printf("flash init\n");
  w25qxx_init(3, 0);
  w25qxx_enable_quad_mode();

  g_lcd_gram0 = (uint32_t *)iomem_malloc(INPUT_WIDTH * INPUT_HEIGHT * 2);
  g_lcd_gram1 = (uint32_t *)iomem_malloc(INPUT_WIDTH * INPUT_HEIGHT * 2);
  g_ai_buf = (uint8_t *)iomem_malloc(INPUT_WIDTH * INPUT_HEIGHT * 3);

#if LOAD_KMODEL_FROM_FLASH
  model_data = (uint8_t *)malloc(KMODEL_SIZE + 255);
  uint8_t *model_data_align =
      (uint8_t *)(((uintptr_t)model_data + 255) & (~255));
  w25qxx_read_data(0xC00000, model_data_align, KMODEL_SIZE, W25QXX_QUAD_FAST);
#else
  uint8_t *model_data_align = model_data;
#endif

  lable_init();

  /* LCD init */
  printf("LCD init\n");
  lcd_init();

#if BOARD_LICHEEDAN
  lcd_set_direction(DIR_YX_RLDU);
#else
  lcd_set_direction(DIR_YX_RLUD);
#endif
  lcd_clear(BLACK);
  lcd_draw_string(136, 70, "DEMO 1", WHITE);
  lcd_draw_string(104, 150, "20 class detection", WHITE);
  /* DVP init */
  printf("DVP init\n");

#if OV5640
  dvp_init(16);
  dvp_set_xclk_rate(50000000);
  dvp_enable_burst();
  dvp_set_output_enable(0, 1);
  dvp_set_output_enable(1, 1);
  dvp_set_image_format(DVP_CFG_RGB_FORMAT);
  dvp_set_image_size(INPUT_WIDTH, INPUT_HEIGHT);
  ov5640_init();
#else
  dvp_init(8);
  dvp_set_xclk_rate(24000000);
  dvp_enable_burst();
  dvp_set_output_enable(0, 1);
  dvp_set_output_enable(1, 1);
  dvp_set_image_format(DVP_CFG_RGB_FORMAT);
  //    dvp_set_image_format(DVP_CFG_YUV_FORMAT);
  dvp_set_image_size(320, 240);
  ov2640_init();
#endif

  dvp_set_ai_addr((uint32_t)g_ai_buf,
                  (uint32_t)(g_ai_buf + INPUT_WIDTH * INPUT_HEIGHT),
                  (uint32_t)(g_ai_buf + INPUT_WIDTH * INPUT_HEIGHT * 2));
  dvp_set_display_addr((uint32_t)g_lcd_gram0);
  dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
  dvp_disable_auto();
  /* DVP interrupt config finish */
  printf("DVP interrupt config finish\n");

  plic_set_priority(IRQN_DVP_INTERRUPT, 1);
  plic_irq_register(IRQN_DVP_INTERRUPT, on_irq_dvp, NULL);
  plic_irq_enable(IRQN_DVP_INTERRUPT);

  g_ram_mux = 0;
  g_dvp_finish_flag = 0;
  dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
  dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);

  /* init kpu */
  if (kpu_load_kmodel(&task, model_data_align) != 0) {
    printf("\nmodel init error\n");
    while (1)
      ;
  }

  // detect_rl.anchor_number = ANCHOR_NUM;
  // detect_rl.anchor = g_anchor;
  // detect_rl.threshold = 0.7;`
  // detect_rl.nms_value = 0.3;
  // region_layer_init(&detect_rl, 10, 7, 125, INPUT_WIDTH, INPUT_HEIGHT);

  detect_rl0.anchor_number = ANCHOR_NUM;
  detect_rl0.anchor = layer0_anchor;
  detect_rl0.threshold = 0.6;
  detect_rl0.nms_value = 0.3;
  region_layer_init(&detect_rl0, 10, 7, 75, INPUT_WIDTH, INPUT_HEIGHT);

  detect_rl1.anchor_number = ANCHOR_NUM;
  detect_rl1.anchor = layer1_anchor;
  detect_rl1.threshold = 0.6;
  detect_rl1.nms_value = 0.3;
  region_layer_init(&detect_rl1, 20, 14, 75, INPUT_WIDTH, INPUT_HEIGHT);

  uint64_t time_last = sysctl_get_time_us();
  uint64_t time_now = sysctl_get_time_us();
  int time_count = 0;

  /* enable global interrupt */
  sysctl_enable_irq();
  /* system start */
  printf("system start\n");

  while (1) {
    /* dvp finish*/
    while (g_dvp_finish_flag == 0)
      ;

    /* start to calculate */
    g_ai_done_flag = 0;
    kpu_run_kmodel(&task, g_ai_buf, DMAC_CHANNEL5, ai_done, NULL);
    while (!g_ai_done_flag)
      ;

    float *output0, *output1;
    size_t output_size0, output_size1;
    // NOTE output_size 是字节， float 是4字节
    kpu_get_output(&task, 0, (uint8_t **)&output0, &output_size0);
    kpu_get_output(&task, 1, (uint8_t **)&output1, &output_size1);

    printf("kpu_get_output done\n");

    // detect_rl.input = output0;
    // region_layer_run(&detect_rl, NULL);
    detect_rl0.input = output0;
    region_layer_run(&detect_rl0, NULL);
    detect_rl1.input = output1;
    region_layer_run(&detect_rl1, NULL);

    /* display pic*/
    g_ram_mux ^= 0x01;
    lcd_draw_picture(0, 0, INPUT_WIDTH, INPUT_HEIGHT,
                     g_ram_mux ? g_lcd_gram0 : g_lcd_gram1);
    g_dvp_finish_flag = 0;

    /* draw boxs */
    // region_layer_draw_boxes(&detect_rl, drawboxes);
    region_layer_draw_boxes(&detect_rl0, drawboxes);
    region_layer_draw_boxes(&detect_rl1, drawboxes);

    time_count++;
    if (time_count % 100 == 0) {
      time_now = sysctl_get_time_us();
      printf("SPF:%fms\n", (time_now - time_last) / 1000.0 / 100);
      time_last = time_now;
    }
  }

  return 0;
}
