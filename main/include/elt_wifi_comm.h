#pragma once

#ifndef ELT_WIFI_COMM_H
#define ELT_WIFI_COMM_H

#include "esp_err.h"
#include "esp_event.h"

#define WIFI_CONNECTED_BIT          (BIT0)

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

void wifi_init(void);
esp_err_t wifi_scan(void);
esp_err_t wifi_connect(const char *ssid, const char *pass);
void wifi_save_credentials(const char *ssid, const char *pass);
bool wifi_load_credentials(char *ssid_out, char * pass_out);
void attempt_wifi_reconnect(void);
bool wifi_connected(void);

#endif //ELT_WIFI_COMM_H

//idf.py menuconfig --> Component config --> LVGL configuration --> Others --> [*] Enable API to take snapshots

//in elt_graphics_ui.h: #define LCD_DRAW_BUFF_HEIGHT                             (40)
//in elt_graphics_ui.h: #define LVGL_PORT_STACK_SIZE                             (32768)
//in elt_graphics_ui.h: #define LVGL_DISPLAY_CONFIG(io, panel)                   .buff_dma = false,
//in elt_graphics_ui.h: #define LVGL_DISPLAY_CONFIG(io, panel)                   .buff_spiram = true,

//in CMakeLists.txt: idf_component_register(SRCS "elt_servo_control.c" "elt_wifi_comm.c" "elt_graphic_ui.c" "elt_shared_data.c" "main.c" "${CMAKE_CURRENT_SOURCE_DIR}/../managed_components/lvgl__lvgl/src/others/snapshot/lv_snapshot.c"
                    //INCLUDE_DIRS "." "${CMAKE_CURRENT_SOURCE_DIR}/../managed_components/lvgl__lvgl/src/others/snapshot" REQUIRES mdns nvs_flash esp_netif esp_wifi ads111x ina219 esp_http_server json)

//#include "lvgl.h"
//#include "esp_lvgl_port.h"

/*
esp_err_t screenshot_get_handler(httpd_req_t *req) {
    lv_obj_t *screen = lv_scr_act();

    uint32_t width = lv_obj_get_width(screen);
    uint32_t height = lv_obj_get_height(screen);
    uint32_t data_size = width * height * sizeof(lv_color_t);

    uint8_t *img_data = (uint8_t *)heap_caps_malloc(data_size, MALLOC_CAP_32BIT | MALLOC_CAP_SPIRAM);
    if (img_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer for screenshot");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    lv_img_dsc_t snapshot = {0};
    snapshot.header.w = width;
    snapshot.header.h = height;
    snapshot.header.cf = LV_COLOR_FORMAT_RGB565;
    snapshot.data_size = data_size;
    snapshot.data = img_data;

    lv_res_t res = lv_snapshot_take_to_buf(screen, LV_COLOR_FORMAT_RGB565, &snapshot, img_data, data_size);
    if (res != LV_RES_OK) {
        ESP_LOGE(TAG, "Failed to take snapshot");
        heap_caps_free(img_data);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"screenshot.raw\"");

    esp_err_t err = httpd_resp_send(req, (const char *)img_data, data_size);

    heap_caps_free(img_data);

    return err;
}
*/

/*
httpd_uri_t screenshot_uri = {
    .uri       = "/screenshot",
    .method    = HTTP_GET,
    .handler   = screenshot_get_handler,
    .user_ctx  = NULL
};
*/

//httpd_register_uri_handler(server, &screenshot_uri);

//full rebuild

//in cmd line: curl http://elt.local/screenshot --output"screenshot.raw"

//in python: python convert_raw.py

//--> screenshot.png über wifi von LVGL screen :)