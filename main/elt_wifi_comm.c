#include "include/elt_wifi_comm.h"
#include "include/elt_shared_data.h"
#include "include/elt_servo_control.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "mdns.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_sntp.h"
#include "cJSON.h"

static const char *TAG = "WiFi";
static httpd_handle_t server = NULL;
static EventGroupHandle_t wifi_event_group;

void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "at.pool.ntp.org");
    esp_sntp_init();
}

void sync_time_task(void *pvParameters) {
    initialize_sntp();

    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1); //setenv("TZ", "UTC-2", 1);
    tzset();
    vTaskDelay(pdMS_TO_TICKS(100));
    time(&now);

    if(xSemaphoreTake(time_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        localtime_r(&now, &timeinfo);
        xSemaphoreGive(time_mutex);
    }
    else {
        ESP_LOGE(TAG, "Failed to acquire time mutex");
    }

    if (retry < retry_count) {
        ESP_LOGI(TAG, "System time was set to %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }

    vTaskDelete(NULL);
}

int get_rssi(void) {
    wifi_ap_record_t info;
    if (esp_wifi_sta_get_ap_info(&info) == ESP_OK) {
        return info.rssi;
    }
    return -127;
}

static esp_err_t api_set_handler(httpd_req_t *req) {
    char buf[128];
    int received = httpd_req_recv(req, buf, MIN(req->content_len, sizeof(buf) - 1));
    if (received <= 0) return ESP_FAIL;

    buf[received] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) return ESP_FAIL;

    cJSON *tracking_val = cJSON_GetObjectItem(json, "tracking");
    cJSON *angles = cJSON_GetObjectItem(json, "servo_angles");

    if (cJSON_IsBool(tracking_val)) {
        if (xSemaphoreTake(tracking_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            tracking = tracking_val->valueint;
            xSemaphoreGive(tracking_mutex);
        }
        else {
            ESP_LOGE(TAG, "Failed to aquire tracking_mutex");
        }
    }

    if (cJSON_IsArray(angles) && cJSON_GetArraySize(angles) == 2) {
        int angle_x = cJSON_GetArrayItem(angles, 0)->valueint;
        int angle_y = cJSON_GetArrayItem(angles, 1)->valueint;

        set_servo_position(0, angle_x);
        set_servo_position(1, angle_y);
    }

    cJSON_Delete(json);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static httpd_uri_t api_set_uri = {
    .uri = "/api/set",
    .method = HTTP_POST,
    .handler = api_set_handler,
    .user_ctx = NULL
};

static esp_err_t api_get_handler(httpd_req_t *req) {
    char buf[512];
    int offset = 0;

    // Acquire locks
    xSemaphoreTake(time_mutex, portMAX_DELAY);
    xSemaphoreTake(shunt_mutex, portMAX_DELAY);
    xSemaphoreTake(ldr_mutex, portMAX_DELAY);
    xSemaphoreTake(servo_mutex, portMAX_DELAY);
    xSemaphoreTake(tracking_mutex, portMAX_DELAY);

    // Format time
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%c", &timeinfo);

    offset += snprintf(buf + offset, sizeof(buf) - offset,
        "{ \"time\": \"%s\", \"shunt_currents\": [%.6f, %.6f], "
        "\"power_estimates\": [%.6f, %.6f], \"ldr_voltages\": [",
        time_str, shunt_current_1, shunt_current_2, power_est_1, power_est_2);

    for (int i = 0; i < NUM_LDRS; i++) {
        offset += snprintf(buf + offset, sizeof(buf) - offset, "%.6f", ldr_voltage[i]);
        if (i < NUM_LDRS - 1) offset += snprintf(buf + offset, sizeof(buf) - offset, ",");
    }

    offset += snprintf(buf + offset, sizeof(buf) - offset,
        "], \"servo_angles\": [%d, %d], \"tracking\": %s }",
        current_servo_angles[0], current_servo_angles[1], tracking ? "true" : "false");

    // Release locks
    xSemaphoreGive(time_mutex);
    xSemaphoreGive(shunt_mutex);
    xSemaphoreGive(ldr_mutex);
    xSemaphoreGive(servo_mutex);
    xSemaphoreGive(tracking_mutex);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_uri_t api_uri = {
    .uri       = "/api/data",
    .method    = HTTP_GET,
    .handler   = api_get_handler,
    .user_ctx  = NULL
};

static esp_err_t root_get_handler(httpd_req_t *req) {
    char buf[2048];
    int offset = 0;

    // Acquire locks
    xSemaphoreTake(time_mutex, portMAX_DELAY);
    xSemaphoreTake(shunt_mutex, portMAX_DELAY);
    xSemaphoreTake(ldr_mutex, portMAX_DELAY);
    xSemaphoreTake(servo_mutex, portMAX_DELAY);
    xSemaphoreTake(tracking_mutex, portMAX_DELAY);

    // Format time
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%c", &timeinfo);

    // Build HTML
    offset += snprintf(buf + offset, sizeof(buf) - offset,
        "<!DOCTYPE html><html><head><title>ELT</title></head><body>"
        "<h1>Embedded Light Tracker</h1>"
        "<p><strong>Current Time:</strong> %s</p>"
        "<p><strong>Shunt Currents:</strong> %.3f mA, %.3f mA</p>"
        "<p><strong>Power Estimates:</strong> %.3f mW, %.3f mW</p>"
        "<p><strong>LDR Voltages:</strong>", time_str,
        shunt_current_1 * 1000.0, shunt_current_2 * 1000.0,
        power_est_1 * 1000.0, power_est_2 * 1000.0);

    for (int i = 0; i < NUM_LDRS; i++) {
        offset += snprintf(buf + offset, sizeof(buf) - offset, " %.3f mV", ldr_voltage[i] * 1000.0);
        if (i < NUM_LDRS - 1) offset += snprintf(buf + offset, sizeof(buf) - offset, ",");
    }

    offset += snprintf(buf + offset, sizeof(buf) - offset,
        "</p><p><strong>Servo Angles:</strong> %d°, %d°</p>"
        "<p><strong>Tracking Enabled:</strong> %s</p>"

        "<form method=\"POST\" action=\"/update\">"
        "<label for=\"servo1\">Servo 1 Angle:</label>"
        "<input type=\"number\" id=\"servo1\" name=\"servo1\" min=\"0\" max=\"%d\" value=\"%d\"><br>"
        "<label for=\"servo2\">Servo 2 Angle:</label>"
        "<input type=\"number\" id=\"servo2\" name=\"servo2\" min=\"0\" max=\"%d\" value=\"%d\"><br>"
        "<label for=\"tracking\">Tracking Enabled:</label>"
        "<input type=\"checkbox\" id=\"tracking\" name=\"tracking\" %s><br>"
        "<input type=\"submit\" value=\"Update\">"
        "</form>"
        "</body></html>",
        current_servo_angles[0], current_servo_angles[1],
        tracking ? "Yes" : "No",
        SERVO_PAN_MAX_DEG,
        current_servo_angles[0], 
        SERVO_TILT_MAX_DEG, current_servo_angles[1],
        tracking ? "checked" : "");

    // Release locks
    xSemaphoreGive(time_mutex);
    xSemaphoreGive(shunt_mutex);
    xSemaphoreGive(ldr_mutex);
    xSemaphoreGive(servo_mutex);
    xSemaphoreGive(tracking_mutex);

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static esp_err_t update_post_handler(httpd_req_t *req) {
    char buf[256];
    int total_len = req->content_len;
    int received = 0;
    while (received < total_len && received < sizeof(buf) - 1) {
        int ret = httpd_req_recv(req, buf + received, sizeof(buf) - 1 - received);
        if (ret <= 0) break;
        received += ret;
    }
    buf[received] = '\0';

    int tracking_val = 0;
    int servo1 = -1;
    int servo2 = -1;

    char *param = strtok(buf, "&");
    while (param != NULL) {
        char *eq = strchr(param, '=');
        if (eq != NULL) {
            *eq = '\0';
            char *key = param;
            char *val = eq + 1;

            if (strcmp(key, "tracking") == 0) {
                if (strcmp(val, "on") == 0) {
                    tracking_val = 1;
                } else {
                    tracking_val = atoi(val);
                }
            } else if (strcmp(key, "servo1") == 0) {
                servo1 = atoi(val);
            } else if (strcmp(key, "servo2") == 0) {
                servo2 = atoi(val);
            }
        }
        param = strtok(NULL, "&");
    }

    if (servo1 < 0) servo1 = 0;
    else if (servo1 > SERVO_PAN_MAX_DEG) servo1 = SERVO_PAN_MAX_DEG;

    if (servo2 < 0) servo2 = 0;
    else if (servo2 > SERVO_TILT_MAX_DEG) servo2 = SERVO_TILT_MAX_DEG;

    // Acquire lock
    xSemaphoreTake(tracking_mutex, pdMS_TO_TICKS(100));

    tracking = tracking_val ? 1 : 0;
    ESP_LOGE(TAG, "Tracking: %d, Tracking_VAL: %d", tracking, tracking_val);
    set_servo_position(0, servo1);
    set_servo_position(1, servo2);

    xSemaphoreGive(tracking_mutex);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static httpd_uri_t update_uri = {
    .uri = "/update",
    .method = HTTP_POST,
    .handler = update_post_handler,
    .user_ctx = NULL
};

void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &api_uri);
        httpd_register_uri_handler(server, &api_set_uri);
        httpd_register_uri_handler(server, &update_uri);
        ESP_LOGI(TAG, "Webserver started");
    } else {
        ESP_LOGE(TAG, "Failed to start webserver");
    }
}
void start_mdns_service(void) {
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("elt"));
    ESP_ERROR_CHECK(mdns_instance_name_set("Embedded Light Tracker"));

    ESP_ERROR_CHECK(mdns_service_add("ELT WebServer", "_http", "_tcp", 80, NULL, 0));
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Started");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected, Reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

        start_mdns_service();

        start_webserver();

        xTaskCreatePinnedToCore(sync_time_task, "sync_time_task", 4096, NULL, 5, NULL,tskNO_AFFINITY);
    }
}

bool wifi_connected(void) {
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

void wifi_init(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

esp_err_t wifi_scan(void) {
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);

    if (mode != WIFI_MODE_STA) {
        ESP_LOGW(TAG, "Wi-Fi not in STA mode, setting...");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_ERROR_CHECK(esp_wifi_start());

    vTaskDelay(pdMS_TO_TICKS(100));

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE
    };

    ESP_LOGI(TAG, "Starting scan...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

    if (xSemaphoreTake(ap_list_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));
        xSemaphoreGive(ap_list_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_FAIL;
    }

    if (xSemaphoreTake(ap_list_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_LOGI(TAG, "Found %d APs", ap_count);
        for (int i = 0; i < ap_count; i++) {
            ESP_LOGI(TAG, "SSID: %s, RSSI: %d", ap_list[i].ssid, ap_list[i].rssi);
        }
        xSemaphoreGive(ap_list_mutex);
    }
    return ESP_OK;
}

esp_err_t wifi_connect(const char *ssid, const char *pass) {
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    ESP_LOGI(TAG, "Connecting to SSID:%s", ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());
    return ESP_OK;
}

void wifi_save_credentials(const char *ssid, const char *pass) {
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("wifi", NVS_READWRITE, &nvs));
    nvs_set_str(nvs, "ssid", ssid);
    nvs_set_str(nvs, "pass", pass);
    nvs_commit(nvs);
    nvs_close(nvs);
}

bool wifi_load_credentials(char *ssid_out, char * pass_out) {
    size_t len;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi", NVS_READONLY, &nvs);
    if (err != ESP_OK) return false;

    len = 32;
    err = nvs_get_str(nvs, "ssid", ssid_out, &len);
    len = 64;
    err |= nvs_get_str(nvs, "pass", pass_out, &len);
    nvs_close(nvs);
    return (err == ESP_OK);
}

void attempt_wifi_reconnect(void) {
    char stored_ssid[32] = {0};
    char stored_pass[64] = {0};

    if (wifi_load_credentials(stored_ssid, stored_pass)) {
        if(xSemaphoreTake(ap_list_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (int i = 0; i < ap_count; i++) {
                if (strcmp((char *)ap_list[i].ssid, stored_ssid) == 0) {
                    ESP_LOGI(TAG, "Attempting to connect to the stored SSID: %s", stored_ssid);
                    esp_err_t result = wifi_connect(stored_ssid, stored_pass);

                    if (result == ESP_OK) {
                        ESP_LOGI(TAG, "Connected to %s", stored_ssid);
                    } else {
                        ESP_LOGE(TAG, "Failed to connect to %s", stored_ssid);
                    }
                    break;
                }
            }
            xSemaphoreGive(ap_list_mutex);
        } else {
            ESP_LOGE(TAG, "Failed to take mutex");
        }
    } else {
        ESP_LOGE(TAG, "No stored credentials found");
    }
}