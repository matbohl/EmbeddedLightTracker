#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "iot_knob.h"
#include "iot_button.h"
#include "esp_lvgl_port_button.h"
#include "esp_lvgl_port_knob.h"

#include "driver/i2c_master.h"


/* LCD size */
#define EXAMPLE_LCD_H_RES   (240)
#define EXAMPLE_LCD_V_RES   (320)

/* LCD settings */
#define EXAMPLE_LCD_SPI_NUM         (SPI2_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ    (20 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS        (8)
#define EXAMPLE_LCD_PARAM_BITS      (8)
#define EXAMPLE_LCD_COLOR_SPACE     (ESP_LCD_COLOR_SPACE_BGR)
#define EXAMPLE_LCD_BITS_PER_PIXEL  (16)
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (50)
#define EXAMPLE_LCD_BL_ON_LEVEL     (1)

/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK       (12)
#define EXAMPLE_LCD_GPIO_MOSI       (11)
#define EXAMPLE_LCD_GPIO_RST        (4)
#define EXAMPLE_LCD_GPIO_DC         (5)
#define EXAMPLE_LCD_GPIO_CS         (6)
#define EXAMPLE_LCD_GPIO_BL         (7)

/* Encoder Pins */
#define GPIO_ENCODER_A              (1)
#define GPIO_ENCODER_B              (2)
#define GPIO_ENCODER_BTN            (3)

static const char *TAG = "EXAMPLE";

// LVGL image declare
LV_IMG_DECLARE(esp_logo)

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;


/*I2C communication*/

#define I2C_TOOL_TIMEOUT_VALUE_MS (50)
static const uint32_t i2c_freq = 100*1000;
i2c_master_bus_handle_t i2c_bus_handle;
int i2c_gpio_sda = 8;
int i2c_gpio_scl = 9;
i2c_port_t i2c_port = I2C_NUM_0;

static const int INA219_chip_addr = 0x45;
static const int ADS1115_chip_addr = 0x48;
static const int BNO085_chip_addr = 0x4B;

static i2c_device_config_t INA219_i2c_dev_conf = {
        .scl_speed_hz = i2c_freq,
        .device_address = INA219_chip_addr,
    };

static i2c_device_config_t ADS1115_i2c_dev_conf = {
        .scl_speed_hz = i2c_freq,
        .device_address = ADS1115_chip_addr,
    };    

static i2c_device_config_t BNO085_i2c_dev_conf = {
        .scl_speed_hz = i2c_freq,
        .device_address = BNO085_chip_addr,
    };

i2c_master_dev_handle_t INA219_i2c_dev_handle;
i2c_master_dev_handle_t ADS1115_i2c_dev_handle;
i2c_master_dev_handle_t BNO085_i2c_dev_handle;

/* LVGL display*/

static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_encoder_indev = NULL;

/*Encoder input*/
const button_config_t encoder_btn_config = {
    .type = BUTTON_TYPE_GPIO,
    .gpio_button_config.active_level = false,
    .gpio_button_config.gpio_num = GPIO_ENCODER_BTN,
};

const knob_config_t encoder_a_b_config = {
    .default_direction = 0,
    .gpio_encoder_a = GPIO_ENCODER_A,
    .gpio_encoder_b = GPIO_ENCODER_B,
};

static esp_err_t app_i2c_init(void)
{
    esp_err_t ret = ESP_OK;

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .sda_io_num = i2c_gpio_sda,
        .scl_io_num = i2c_gpio_scl,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    if(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle) != ESP_OK){
        ESP_LOGE(TAG, "I2C bus create error");
        return ESP_FAIL;
    }
    
   if(i2c_master_bus_add_device(i2c_bus_handle, &INA219_i2c_dev_conf, &INA219_i2c_dev_handle) != ESP_OK){
        ESP_LOGE(TAG, "I2C INA219 device create error");
        return ESP_FAIL;
    }
   if(i2c_master_bus_add_device(i2c_bus_handle, &ADS1115_i2c_dev_conf, &ADS1115_i2c_dev_handle) != ESP_OK){
        ESP_LOGE(TAG, "I2C ADS1115 device create error");
        return ESP_FAIL;
    }
    if(i2c_master_bus_add_device(i2c_bus_handle, &BNO085_i2c_dev_conf, &BNO085_i2c_dev_handle) != ESP_OK){
          ESP_LOGE(TAG, "I2C BNO085 device create error");
          return ESP_FAIL;
    }


    return ret;
}

static int do_i2cdetect_cmd(void)
{
    lvgl_port_lock(0);
    
    uint8_t buf1[1] = {0x00};
    uint8_t buffer[2];
    esp_err_t ret = i2c_master_transmit_receive(INA219_i2c_dev_handle, buf1, sizeof(buf1), buffer, 2, -1);
    printf("INA219: %02x %02x\n", buffer[0], buffer[1]);
    
    uint8_t buf2[1] = {0x01};
    ret = i2c_master_transmit_receive(ADS1115_i2c_dev_handle, buf2, sizeof(buf2), buffer, 2, -1);
    printf("ADS1115: %02x %02x\n", buffer[0], buffer[1]);

    /*uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            esp_err_t ret = i2c_master_probe(i2c_bus_handle, address, I2C_TOOL_TIMEOUT_VALUE_MS);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }
    
    */
    return 0;
    lvgl_port_unlock();
}

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD backlight */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
        .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
        .cs_gpio_num = EXAMPLE_LCD_GPIO_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
        .color_space = EXAMPLE_LCD_COLOR_SPACE,
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL));

    return ret;

err:
    if (lcd_panel) {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io) {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(EXAMPLE_LCD_SPI_NUM);
    return ret;
}

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,         /* LVGL task priority */
        .task_stack = 4096,         /* LVGL task stack size */
        .task_affinity = 0,        /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
        .timer_period_ms = 5        /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    const lvgl_port_encoder_cfg_t encoder = {
        .disp = lvgl_disp,
        .encoder_a_b = &encoder_a_b_config,
        .encoder_enter = &encoder_btn_config,
    };
    
    lvgl_encoder_indev = lvgl_port_add_encoder(&encoder);

    
    lv_indev_set_type(lvgl_encoder_indev,LV_INDEV_TYPE_ENCODER);
    lv_group_t * InputDeviceGroup = lv_group_create();
    lv_group_set_default(InputDeviceGroup);
    lv_indev_set_group(lvgl_encoder_indev, InputDeviceGroup);

    return ESP_OK;
}

static void _app_button_cb(lv_event_t *e)
{
    lv_disp_rotation_t rotation = lv_disp_get_rotation(lvgl_disp);
    rotation++;
    if (rotation > LV_DISPLAY_ROTATION_270) {
        rotation = LV_DISPLAY_ROTATION_0;
    }

    /* LCD HW rotation */
    lv_disp_set_rotation(lvgl_disp, rotation);
}

static void _app_button2_cb(lv_event_t *e)
{
    do_i2cdetect_cmd();
}

static void app_main_display(void)
{
    lv_obj_t *scr = lv_scr_act();
    

    /* Task lock */
    lvgl_port_lock(0);

    /* Create image 
    lv_obj_t *img_logo = lv_img_create(scr);
    lv_img_set_src(img_logo, &esp_logo);
    lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 50);
    */ 
    //

    /* Label */
    lv_obj_t *label = lv_label_create(scr);
    lv_obj_set_width(label, EXAMPLE_LCD_H_RES);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
#if LVGL_VERSION_MAJOR == 8
    lv_label_set_recolor(label, true);
    lv_label_set_text(label, "#FF0000  Embedded Light Tracker#\n#FF9400 "LV_SYMBOL_WARNING" For simplier initialization, use BSP "LV_SYMBOL_WARNING" #");
#else
    lv_label_set_text(label, "Embedded Light Tracker \n "LV_SYMBOL_WARNING" ESP-IDF is cursed "LV_SYMBOL_WARNING);
#endif
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);

    /* Button A */
    lv_obj_t *btn1 = lv_btn_create(scr);
    label = lv_label_create(btn1);
    lv_label_set_text_static(label, "Button A");
    lv_obj_align(btn1, LV_ALIGN_CENTER, -50, 0);
    lv_obj_add_event_cb(btn1, _app_button_cb, LV_EVENT_CLICKED, NULL);

    /* Button B */
    lv_obj_t *btn2 = lv_btn_create(scr);
    label = lv_label_create(btn2);
    lv_label_set_text_static(label, "Button B");
    lv_obj_align(btn2, LV_ALIGN_CENTER, -50, 50);
    lv_obj_add_event_cb(btn2, _app_button2_cb, LV_EVENT_CLICKED, NULL);
    
    /* Task unlock */
    lvgl_port_unlock();
}

void app_main(void)
{
    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_i2c_init());

    ESP_ERROR_CHECK(app_lcd_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* Show LVGL objects */
    app_main_display();
}
