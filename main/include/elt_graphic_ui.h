#pragma once

#ifndef ELT_GRAPHIC_UI_H
#define ELT_GRAPHIC_UI_H

#include "esp_err.h"

#define LCD_H_RES                                        (240)
#define LCD_V_RES                                        (320)

#define LCD_SPI_NUM                                      (SPI2_HOST)
#define LCD_PIXEL_CLK_HZ                                 (50 * 1000 * 1000)
#define LCD_CMD_BITS                                     (8)
#define LCD_PARAM_BITS                                   (8)
#define LCD_COLOR_SPACE                                  (ESP_LCD_COLOR_SPACE_RGB)
#define LCD_BITS_PER_PIXEL                               (16)
#define LCD_DRAW_BUFF_DOUBLE                             (1)
#define LCD_DRAW_BUFF_HEIGHT                             (20)
#define LCD_BL_ON_LEVEL                                  (1)
#define LCD_SPI_MODE                                     (0)
#define LCD_TRANSACTION_QUEUE_DEPTH                      (10)

#define LCD_GPIO_SCLK                                    (12)
#define LCD_GPIO_MOSI                                    (11)
#define LCD_GPIO_RST                                     (4)
#define LCD_GPIO_DC                                      (5)
#define LCD_GPIO_CS                                      (6)
#define LCD_GPIO_BL                                      (7)

#define GPIO_ENCODER_A                                   (1)
#define GPIO_ENCODER_B                                   (2)
#define GPIO_ENCODER_BTN                                 (3)
#define GPIO_ENCODER_BTN_ACTIVE                          (false)
#define GPIO_ENCODER_DEFAULT_DIR                         (0)

#define LVGL_PORT_TASK_PRIO                              (4)
#define LVGL_PORT_STACK_SIZE                             (8192)
#define LVGL_PORT_TASK_AFFINITY                          (1)
#define LVGL_PORT_TASK_MAX_SLEEP_MS                      (500)
#define LVGL_PORT_TIMER_PERIOD_MS                        (5)

#define LCD_SPI_BUS_CONFIG                               { \
                                                             .sclk_io_num = LCD_GPIO_SCLK, \
                                                            .mosi_io_num = LCD_GPIO_MOSI, \
                                                            .miso_io_num = GPIO_NUM_NC, \
                                                            .quadwp_io_num = GPIO_NUM_NC, \
                                                            .quadhd_io_num = GPIO_NUM_NC, \
                                                            .max_transfer_sz = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t) \
                                                         }

#define LCD_LCD_PANEL_IO_SPI_CONFIG                      { \
                                                            .dc_gpio_num = LCD_GPIO_DC, \
                                                            .cs_gpio_num = LCD_GPIO_CS, \
                                                            .pclk_hz = LCD_PIXEL_CLK_HZ, \
                                                            .lcd_cmd_bits = LCD_CMD_BITS, \
                                                            .lcd_param_bits = LCD_PARAM_BITS, \
                                                            .spi_mode = LCD_SPI_MODE, \
                                                            .trans_queue_depth = LCD_TRANSACTION_QUEUE_DEPTH, \
                                                         }
                                    
#define LCD_PANEL_DEVICE_CONFIG                          { \
                                                            .reset_gpio_num = LCD_GPIO_RST, \
                                                            .color_space = LCD_COLOR_SPACE, \
                                                            .bits_per_pixel = LCD_BITS_PER_PIXEL, \
                                                         }

#define BUTTON_CONFIG                                    { \
                                                            .type = BUTTON_TYPE_GPIO, \
                                                            .gpio_button_config.active_level = GPIO_ENCODER_BTN_ACTIVE, \
                                                            .gpio_button_config.gpio_num = GPIO_ENCODER_BTN, \
                                                         }

#define KNOB_CONFIG                                      { \
                                                            .default_direction = GPIO_ENCODER_DEFAULT_DIR, \
                                                            .gpio_encoder_a = GPIO_ENCODER_A, \
                                                            .gpio_encoder_b = GPIO_ENCODER_B, \
                                                         }

#define BL_GPIO_CONFIG                                   { \
                                                            .mode = GPIO_MODE_OUTPUT, \
                                                            .pin_bit_mask = (1ULL << LCD_GPIO_BL), \
                                                         }

#define LVGL_PORT_CONFIG                                 { \
                                                            .task_priority = LVGL_PORT_TASK_PRIO, \
                                                            .task_stack = LVGL_PORT_STACK_SIZE, \
                                                            .task_affinity = LVGL_PORT_TASK_AFFINITY, \
                                                            .task_max_sleep_ms = LVGL_PORT_TASK_MAX_SLEEP_MS, \
                                                            .timer_period_ms = LVGL_PORT_TIMER_PERIOD_MS, \
                                                         }

#define LVGL_DISPLAY_CONFIG(io, panel)                   { \
                                                            .io_handle = io, \
                                                            .panel_handle = panel, \
                                                            .buffer_size = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT, \
                                                            .double_buffer = LCD_DRAW_BUFF_DOUBLE, \
                                                            .hres = LCD_H_RES, \
                                                            .vres = LCD_V_RES, \
                                                            .monochrome = false, \
                                                            .rotation = { \
                                                                            .swap_xy = false, \
                                                                            .mirror_x = true, \
                                                                            .mirror_y = true, \
                                                                        }, \
                                                            .flags =    { \
                                                                            .buff_dma = true, \
                                                                            .buff_spiram = false, \
                                                                            .swap_bytes = true, \
                                                                        }, \
                                                            /* LVGL version conditional */ \
                                                            LVGL_DISPLAY_CONFIG_EXTRA \
                                                         }

#if LVGL_VERSION_MAJOR >= 9
#define LVGL_DISPLAY_CONFIG_EXTRA .color_format = LV_COLOR_FORMAT_RGB565,
#else
#define LVGL_DISPLAY_CONFIG_EXTRA
#endif


#define LVGL_ENCODER_CONFIG(disp_ptr, ab_ptr, enter_ptr) { \
                                                            .disp = disp_ptr, \
                                                            .encoder_a_b = ab_ptr, \
                                                            .encoder_enter = enter_ptr \
                                                         }

esp_err_t app_lcd_init(void);
void lvgl_task_entry(void *pvParameter);

#endif //ELT_GRAPHIC_UI_H