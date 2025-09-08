#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "iot_button.h"
#include "iot_knob.h"
#include "esp_lvgl_port_button.h"
#include "esp_lvgl_port_knob.h"
#include <string.h>
#include "include/elt_servo_control.h"
#include "include/elt_shared_data.h"
#include "include/elt_graphic_ui.h"
#include "include/elt_wifi_comm.h"

static const char *TAG = "GRAPHIC";
#define MAX_SSID_LENGTH 33
#define BUFFER_SIZE 16

typedef struct {
    lv_obj_t * arc;
    lv_obj_t * label;
} angle_ui_t;

static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

static char buffer[BUFFER_SIZE];

static char selected_ssid[MAX_SSID_LENGTH] = {0};

static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_encoder_indev = NULL;
static lv_obj_t *menu = NULL;
static lv_group_t *menu_group  = NULL;
static lv_obj_t *main_page = NULL;
static lv_obj_t * menu_screen = NULL;

static const button_config_t encoder_btn_config = BUTTON_CONFIG;
static const knob_config_t encoder_a_b_config = KNOB_CONFIG;

/* LCD Init functions*/
esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGD(TAG, "Initialize LCD backlight");
    gpio_config_t bk_gpio_config = BL_GPIO_CONFIG;
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = LCD_SPI_BUS_CONFIG;
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = LCD_LCD_PANEL_IO_SPI_CONFIG;
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = LCD_PANEL_DEVICE_CONFIG;
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);
    esp_lcd_panel_invert_color(lcd_panel, true);
    
    ESP_ERROR_CHECK(gpio_set_level(LCD_GPIO_BL, LCD_BL_ON_LEVEL));

    return ret;

    err:
        if (lcd_panel) {
            esp_lcd_panel_del(lcd_panel);
        }
        if (lcd_io) {
            esp_lcd_panel_io_del(lcd_io);
        }
        spi_bus_free(LCD_SPI_NUM);
        return ret;
}

static esp_err_t app_lvgl_init(void)
{
    const lvgl_port_cfg_t lvgl_cfg = LVGL_PORT_CONFIG;
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = LVGL_DISPLAY_CONFIG(lcd_io, lcd_panel);
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    const lvgl_port_encoder_cfg_t encoder = LVGL_ENCODER_CONFIG(lvgl_disp, &encoder_a_b_config, &encoder_btn_config);

    lvgl_encoder_indev = lvgl_port_add_encoder(&encoder);

    lv_indev_set_type(lvgl_encoder_indev,LV_INDEV_TYPE_ENCODER);
    lv_group_t * InputDeviceGroup = lv_group_create();
    lv_group_set_default(InputDeviceGroup);
    lv_indev_set_group(lvgl_encoder_indev, InputDeviceGroup);

    return ESP_OK;
}

/* LVGL CALLBACKS*/
static void _update_table_cb(lv_timer_t *timer)
{   
    lvgl_port_lock(0);
    lv_obj_t * table = lv_timer_get_user_data(timer);
    char current1[16], power1[16], current2[16], power2[16];

    if(xSemaphoreTake(shunt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        snprintf(current1, sizeof(current1), "%0.2f", shunt_current_1*1000);
        snprintf(power1, sizeof(power1), "%0.2f", power_est_1 * 1000);
        snprintf(current2, sizeof(current2), "%0.2f", shunt_current_2*1000);
        snprintf(power2, sizeof(power2), "%0.2f", power_est_2 * 1000);
        xSemaphoreGive(shunt_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to aquire shunt_mutex");
    }
    lv_table_set_cell_value(table, 1, 1, current1);
    lv_table_set_cell_value(table, 1, 2, power1);
    lv_table_set_cell_value(table, 2, 1, current2);
    lv_table_set_cell_value(table, 2, 2, power2);

    lv_obj_invalidate(table);
    lvgl_port_unlock();
}

static void _tracking_button_cb(lv_event_t *e)
{
    if (xSemaphoreTake(tracking_mutex , pdMS_TO_TICKS(100)) == pdTRUE) {
        tracking = !tracking;
        xSemaphoreGive(tracking_mutex);
    }
    else {
        ESP_LOGE(TAG, "Failed to aquire tracking_mutex");
    }
}

static void _arc_1_value_changed_cb(lv_event_t *e)
{
    angle_ui_t * ui = lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * arc = ui->arc;
    static bool busy = false;
    if(code == LV_EVENT_KEY && !busy) {
        busy = true;
        uint32_t key = lv_event_get_key(e);
        int value = lv_arc_get_value(arc);

        if(key == LV_KEY_RIGHT)
            value++;
        else if(key == LV_KEY_LEFT)
            value--;
        //ESP_LOGI("SERVO PAN", "Val before clamp: %d", value);
        value = LV_CLAMP(lv_arc_get_min_value(arc), value, lv_arc_get_max_value(arc));

        lv_arc_set_value(arc, value);
        busy = false;
        return;
    }
    if (code == LV_EVENT_VALUE_CHANGED ) // || code == LV_EVENT_KEY)
    {
        int val = lv_arc_get_value(arc);
        lv_label_set_text_fmt(ui->label, "%d°", val);
        lv_arc_rotate_obj_to_angle(arc, ui->label, -25);
        //ESP_LOGI("SERVO PAN", "Angle: %d", val);
        set_servo_position(0, val);
    }
}

static void _arc_2_value_changed_cb(lv_event_t *e)
{
    angle_ui_t * ui = lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * arc = ui->arc;
    static bool busy = false;
    if(code == LV_EVENT_KEY && !busy) {
        busy = true;
        uint32_t key = lv_event_get_key(e);
        int value = lv_arc_get_value(arc);

        if(key == LV_KEY_RIGHT)
            value++;
        else if(key == LV_KEY_LEFT)
            value--;
        //ESP_LOGI("SERVO TILT", "Val before clamp: %d", value);
        value = LV_CLAMP(lv_arc_get_min_value(arc), value, lv_arc_get_max_value(arc));
        lv_arc_set_value(arc, value);
        busy = false;
        return;
    }
    if (code == LV_EVENT_VALUE_CHANGED) // || code == LV_EVENT_KEY)
    {
        int val = lv_arc_get_value(arc);
        lv_label_set_text_fmt(ui->label, "%d°", val);
        lv_arc_rotate_obj_to_angle(arc, ui->label, -25);
        //ESP_LOGI("SERVO PAN", "Angle: %d", val);
        set_servo_position(1, val);
    }
}

void keyboard_post_connect_cb(void *param) {
    lv_indev_t * encoder = lv_indev_get_next(NULL);

    while( encoder) {
        if (lv_indev_get_type(encoder) == LV_INDEV_TYPE_ENCODER) {
            lv_indev_set_group(encoder, menu_group);
            break;
        }
        encoder = lv_indev_get_next(encoder);
    }

    lv_obj_t *main_page = lv_menu_get_cur_main_page(menu);
    if (main_page) {
        lv_obj_t *first_item = lv_obj_get_child(main_page, 0);
        if (first_item) {
            lv_group_focus_obj(first_item);
        }
    }
}

void keyboard_event_cb(lv_event_t *e) {
    lv_obj_t * ta = lv_event_get_user_data(e);
    const char *pw = lv_textarea_get_text(ta);

    wifi_connect(selected_ssid, pw);
    wifi_save_credentials(selected_ssid, pw);

    ESP_LOGI("Connect", "Connecting to %s, with password %s", selected_ssid, pw);

    lv_scr_load_anim(menu_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
    lv_menu_set_page(menu, main_page);
    
    lv_async_call(keyboard_post_connect_cb, NULL);
}

void open_password_screen(const char *ssid) {
    lv_obj_t * screen  = lv_obj_create(NULL);
    lv_scr_load_anim(screen,LV_SCR_LOAD_ANIM_MOVE_LEFT,300,0,false);

    lv_group_t * kb_group = lv_group_create();

    lv_indev_t *encoder = lv_indev_get_next(NULL);
    while (encoder) {
        if (lv_indev_get_type(encoder) == LV_INDEV_TYPE_ENCODER) {
            lv_indev_set_group(encoder, kb_group);
            break;
        }
        encoder = lv_indev_get_next(encoder);
    }

    lv_obj_t * ta = lv_textarea_create(screen);
    lv_obj_set_width(ta, 206);
    lv_obj_set_height(ta, 40);
    lv_textarea_set_placeholder_text(ta, "Password");
    lv_obj_align(ta, LV_ALIGN_TOP_MID, 0, 45);

    lv_obj_t * kb = lv_keyboard_create(screen);
    lv_obj_set_size(kb, 230, 180);
    lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_keyboard_set_textarea(kb, ta);
    lv_group_add_obj(kb_group, kb);

    lv_obj_add_event_cb(kb, keyboard_event_cb, LV_EVENT_READY, ta);

    lv_group_focus_obj(kb);
}

void ssid_select_cb(lv_event_t *e) {
    lv_obj_t * dd = lv_event_get_target(e);
    uint16_t selected = lv_dropdown_get_selected(dd);

    if (selected == 0) {
        return;
    }

    uint32_t index = selected - 1;
    strncpy(selected_ssid, (char *)ap_list[index].ssid, sizeof(selected_ssid));
    open_password_screen(selected_ssid);
}

static void update_time(lv_timer_t *timer) {
    lv_obj_t *time_label = (lv_obj_t *)lv_timer_get_user_data(timer);
    if (!time_label) return;

    time_t now;
    if (xSemaphoreTake(time_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        time(&now);
        localtime_r(&now, &timeinfo);
        xSemaphoreGive(time_mutex);   
    } else {
        ESP_LOGE("TIME", "Failed to acquire time mutex");
        return;
    }
    
        char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);

    lv_label_set_text(time_label, strftime_buf);
    lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 0);
}

/* LVGL DISPLAY*/
static void app_main_display(void)
{   
    //screen
    menu_screen = lv_obj_create(NULL);
    lv_scr_load(menu_screen);

    //style
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_color_white());
    lv_style_set_text_color(&style, lv_color_black());
    lv_obj_add_style(menu_screen, &style, 0);

    //menu group
    menu_group = lv_group_create();
    lv_group_set_wrap(menu_group, true);

    //arc style
    static lv_style_t arc_focus_style;
    lv_style_init(&arc_focus_style);
    lv_style_set_outline_width(&arc_focus_style, 3);
    lv_style_set_outline_color(&arc_focus_style, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_outline_pad(&arc_focus_style, 4);

    //encoder indev setup
    lv_indev_t *encoder = lv_indev_get_next(NULL);
    while (encoder) {
        if (lv_indev_get_type(encoder) == LV_INDEV_TYPE_ENCODER) {
            lv_indev_set_group(encoder, menu_group);
            break;
        }
        encoder = lv_indev_get_next(encoder);
    }
    
    //image
    //    lv_obj_t *img_logo = lv_img_create(scr);
    //    lv_img_set_src(img_logo, &esp_logo);
    //    lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 200);

    //label
    lv_obj_t *label = lv_label_create(menu_screen);
    lv_obj_set_width(label, LCD_H_RES);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "Embedded Light Tracker ");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);

    //create menu
    menu = lv_menu_create(menu_screen);
    lv_obj_set_size(menu, lv_disp_get_hor_res(lvgl_disp)-20,lv_disp_get_ver_res(lvgl_disp)-20);
    lv_obj_set_pos(menu,15,50);
    lv_obj_clear_flag(menu, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(menu, LV_OBJ_FLAG_CLICK_FOCUSABLE);

    lv_obj_t * cont;

    //sub page automatic
    lv_obj_t * sub_page1 = lv_menu_page_create(menu,"Automatic");
    cont = lv_menu_cont_create(sub_page1);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Tracking");
    lv_obj_t * sw = lv_switch_create(cont);
    lv_group_add_obj(menu_group, sw);
    lv_obj_add_event_cb(sw, _tracking_button_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    //sub page servo pan
    lv_obj_t * sub_page2 = lv_menu_page_create(menu,"Servo Pan");
    cont = lv_menu_cont_create(sub_page2);

    angle_ui_t * pan_ui = malloc(sizeof(angle_ui_t));
    
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_size(cont, LV_PCT(100), LV_SIZE_CONTENT);

    lv_obj_set_width(cont, lv_pct(100));
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);

    pan_ui->arc = lv_arc_create(cont);
    lv_group_add_obj(menu_group, pan_ui->arc);
    lv_obj_add_style(pan_ui->arc, &arc_focus_style, LV_STATE_FOCUSED);
    lv_arc_set_bg_angles(pan_ui->arc, 0, 180);
    lv_arc_set_range(pan_ui->arc, 0, 178);
    lv_arc_set_value(pan_ui->arc, 90);
    lv_obj_set_size(pan_ui->arc,150,150);
    lv_arc_set_rotation(pan_ui->arc, 180);

    lv_obj_add_flag(pan_ui->arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(pan_ui->arc, LV_OBJ_FLAG_CLICK_FOCUSABLE);

    pan_ui->label = lv_label_create(pan_ui->arc);
    lv_label_set_text_fmt(pan_ui->label, "%d°", 90);
    lv_obj_center(pan_ui->label);

    lv_obj_add_event_cb(pan_ui->arc, _arc_1_value_changed_cb, LV_EVENT_ALL, pan_ui);

    //sub page servo tilt
    lv_obj_t * sub_page3 = lv_menu_page_create(menu,"Servo Tilt");
    cont = lv_menu_cont_create(sub_page3);

    angle_ui_t * tilt_ui = malloc(sizeof(angle_ui_t));

    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_size(cont, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_width(cont, lv_pct(100));
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);

    tilt_ui->arc = lv_arc_create(cont);
    lv_group_add_obj(menu_group, tilt_ui->arc);
    lv_obj_add_style(tilt_ui->arc, &arc_focus_style, LV_STATE_FOCUSED);

    lv_arc_set_angles(tilt_ui->arc, 0, 180);
    //lv_arc_set_rotation(tilt_ui->arc, 90);
    lv_arc_set_range(tilt_ui->arc, 0, 148);
    lv_arc_set_value(tilt_ui->arc, 45);
    lv_obj_set_size(tilt_ui->arc,150,150);

    lv_obj_add_flag(tilt_ui->arc, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(tilt_ui->arc, LV_OBJ_FLAG_CLICK_FOCUSABLE);

    tilt_ui->label = lv_label_create(tilt_ui->arc);
    lv_label_set_text_fmt(tilt_ui->label, "%d°", 45);
    lv_obj_center(tilt_ui->label);

    lv_obj_add_event_cb(tilt_ui->arc, _arc_2_value_changed_cb, LV_EVENT_ALL, tilt_ui);
    lv_obj_center(tilt_ui->label);
    
    //sub page solar data
    lv_obj_t * sub_page4 = lv_menu_page_create(menu,"Solar Power");
    cont = lv_menu_cont_create(sub_page4);
    lv_obj_t * table = lv_table_create(cont);
    lv_obj_clear_flag(table, LV_OBJ_FLAG_SCROLL_ON_FOCUS | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_CHAIN);    

    lv_obj_set_style_text_align(table, LV_TEXT_ALIGN_RIGHT, 0);

    lv_obj_set_width (table, 180);
    lv_obj_set_height(table, 200);
    lv_table_set_col_cnt(table, 3);
    lv_table_set_col_width(table, 0, 45);
    lv_table_set_col_width(table, 1, 60);
    lv_table_set_col_width(table, 2, 75);
    lv_table_set_row_cnt(table, 3);
    lv_table_set_cell_value(table, 0, 0, "Dev");
    lv_table_set_cell_value(table, 1, 0, "1.");
    lv_table_set_cell_value(table, 2, 0, "2.");
    lv_table_set_cell_value(table, 0, 1, "mA");
    lv_table_set_cell_value(table, 0, 2, "mW e");

    snprintf(buffer, sizeof(buffer), "%0.2f", shunt_current_1);
    lv_table_set_cell_value(table, 1, 1, buffer);
    snprintf(buffer, sizeof(buffer), "%0.2f", power_est_1);
    lv_table_set_cell_value(table, 1, 2, buffer);
    snprintf(buffer, sizeof(buffer), "%0.2f", shunt_current_2);
    lv_table_set_cell_value(table, 2, 1, buffer);
    snprintf(buffer, sizeof(buffer), "%0.2f", power_est_2);
    lv_table_set_cell_value(table, 2, 2, buffer);

    lv_timer_create(_update_table_cb, 1000, table);

    //sub page about
    lv_obj_t * sub_page5 = lv_menu_page_create(menu,"About");
    cont = lv_menu_cont_create(sub_page5);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Version 1.0.1");
    lv_obj_t * time_label = lv_label_create(cont);
    lv_label_set_text(time_label, "Time: --:--");
    lv_obj_align(time_label, LV_ALIGN_CENTER, 20,0 );
    lv_obj_t * info_label = lv_label_create(cont);
    lv_label_set_text(info_label, "BA Project 2025");
    lv_obj_align(info_label, LV_ALIGN_CENTER, 30,0 );
    lv_obj_t * name_label = lv_label_create(cont);
    lv_label_set_text(name_label, "Mathias Bohle");
    lv_obj_align(name_label, LV_ALIGN_CENTER, 40,0 );
    lv_obj_t * sign_label = lv_label_create(cont);
    lv_label_set_text(sign_label, "EL23B064");
    lv_obj_align(sign_label, LV_ALIGN_CENTER, 50,0 );

    lv_timer_create(update_time, 1000, time_label);

    //sub page wifi
    lv_obj_t * sub_page_wifi = lv_menu_page_create(menu, "Wi-Fi Settings");

    cont = lv_menu_cont_create(sub_page_wifi);
    lv_obj_add_flag(cont, LV_OBJ_FLAG_CLICKABLE);
    label = lv_label_create(cont);
    lv_label_set_text(label, "SSID");
    lv_obj_t * wifi_dd = lv_dropdown_create(cont);
    lv_obj_add_flag(wifi_dd, LV_OBJ_FLAG_CLICKABLE);
    lv_group_add_obj(menu_group, wifi_dd);
    lv_obj_set_width(wifi_dd, 170);

    if(xSemaphoreTake(ap_list_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        char options[ap_count * 64 + 32];
        strcpy(options, "Select SSID\n");

        for (int i = 0; i < ap_count; i++) {
            char option[64];
            snprintf(option, sizeof(option), "%s (%d dBm)\n", (char *)ap_list[i].ssid, ap_list[i].rssi);
            strcat(options, option);
        }
        lv_dropdown_set_options(wifi_dd, options);
        xSemaphoreGive(ap_list_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        char options[64];
        strcpy(options, "No SSIDs found.\n");
        lv_dropdown_set_options(wifi_dd, options);
    }
    
    lv_dropdown_set_selected(wifi_dd, 0);
    lv_obj_add_event_cb(wifi_dd, ssid_select_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Create main page and add menu items
    main_page = lv_menu_page_create(menu, "MENU");
    
    //automatic control menu item
    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Light Tracking");
    lv_group_add_obj(menu_group, cont);
    lv_menu_set_load_page_event(menu, cont, sub_page1);

    //servo Pan menu item
    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Servo Pan");
    lv_group_add_obj(menu_group, cont);
    lv_menu_set_load_page_event(menu, cont, sub_page2);

    //servo Tilt menu item
    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Servo Tilt");
    lv_group_add_obj(menu_group, cont);
    lv_menu_set_load_page_event(menu, cont, sub_page3);
    
    //current Sensors menu item
    cont = lv_menu_cont_create(main_page);
    lv_obj_add_flag(cont, LV_OBJ_FLAG_CLICKABLE);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Current Sensors");
    lv_group_add_obj(menu_group, cont);
    lv_menu_set_load_page_event(menu, cont, sub_page4);

    //about menu item
    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "About");
    lv_group_add_obj(menu_group, cont);
    lv_menu_set_load_page_event(menu, cont, sub_page5);

    //wifi settings menu item
    cont = lv_menu_cont_create(main_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Wi-Fi Settings");
    lv_group_add_obj(menu_group, cont);
    lv_menu_set_load_page_event(menu, cont, sub_page_wifi);
    
    //make sure back button is accessible
    lv_obj_t *back_btn = lv_menu_get_main_header_back_button(menu);
    if (back_btn) {
        lv_group_add_obj(menu_group, back_btn);
        lv_obj_clear_flag(back_btn, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(back_btn, LV_OBJ_FLAG_CLICKABLE);
    }
    
    //set the main page as root page
    lv_menu_set_page(menu, main_page);
    //focus on first sub_page
    lv_group_focus_obj(lv_obj_get_child(main_page, 0));
}

void lvgl_task_entry(void *arg)
{
    ESP_ERROR_CHECK(app_lvgl_init());

    lvgl_port_lock(0);
    app_main_display();
    lvgl_port_unlock();

    vTaskDelete(NULL);
}