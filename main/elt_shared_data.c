#include "include/elt_shared_data.h"
#include "esp_log.h"

static const char *TAG = "SHARED_DATA";

wifi_ap_record_t ap_list[MAX_AP_NUM];
uint16_t ap_count = 0;
SemaphoreHandle_t ap_list_mutex = NULL;

struct tm timeinfo;
SemaphoreHandle_t time_mutex = NULL;

float shunt_current_1 = 0.0;
float shunt_current_2 = 0.0;
float power_est_1 = 0.0;
float power_est_2 = 0.0;
SemaphoreHandle_t shunt_mutex = NULL;

float ldr_voltage[NUM_LDRS] = {0.0, 0.0, 0.0, 0.0};
SemaphoreHandle_t ldr_mutex = NULL;

int current_servo_angles[NUM_SERVOS] = {0, 0};
SemaphoreHandle_t servo_mutex = NULL;

bool tracking = false;
SemaphoreHandle_t tracking_mutex = NULL;

void shared_data_init(void) {
    ap_list_mutex = xSemaphoreCreateMutex();
    time_mutex = xSemaphoreCreateMutex();
    shunt_mutex = xSemaphoreCreateMutex();
    ldr_mutex = xSemaphoreCreateMutex();
    servo_mutex = xSemaphoreCreateMutex();
    tracking_mutex = xSemaphoreCreateMutex();

    if (ap_list_mutex == NULL || time_mutex == NULL || shunt_mutex == NULL || ldr_mutex == NULL || servo_mutex == NULL || tracking_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
    }
}