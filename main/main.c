#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string.h>
#include "include/elt_servo_control.h"
#include "include/elt_wifi_comm.h"
#include "include/elt_graphic_ui.h"
#include "include/elt_shared_data.h"

static const char *TAG = "LIGHT TRACKER";

void app_main(void)
{
    shared_data_init();

    ESP_ERROR_CHECK(app_lcd_init());

    wifi_init();
    wifi_scan();

    ESP_ERROR_CHECK(esp_wifi_start());
    attempt_wifi_reconnect();

    xTaskCreatePinnedToCore(lvgl_task_entry, "lvgl_task", 8192, NULL, 4, NULL,1);

    xTaskCreatePinnedToCore(servo_control_task, "servo_control_task", 4096, NULL, 3, NULL,1);
    
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG,"Main loop");
    }
}
