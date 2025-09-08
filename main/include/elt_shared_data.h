#pragma once

#ifndef ELT_SHARED_DATA_H
#define ELT_SHARED_DATA_H

#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <time.h>

#define MAX_AP_NUM              (5)
#define NUM_SERVOS              (2)
#define NUM_LDRS                (4) 

extern wifi_ap_record_t ap_list[MAX_AP_NUM];
extern uint16_t ap_count;

extern SemaphoreHandle_t ap_list_mutex;

extern struct tm timeinfo;
extern SemaphoreHandle_t time_mutex;

extern float shunt_current_1;
extern float shunt_current_2;
extern float power_est_1;
extern float power_est_2;
extern SemaphoreHandle_t shunt_mutex;

extern float ldr_voltage[NUM_LDRS];
extern SemaphoreHandle_t ldr_mutex;

extern int current_servo_angles[NUM_SERVOS];
extern SemaphoreHandle_t servo_mutex;

extern bool tracking;
extern SemaphoreHandle_t tracking_mutex;

void shared_data_init(void);

#endif //ELT_SHARED_DATA_H