#pragma once

#ifndef ELT_SERVO_CONTROL_H
#define ELT_SERVO_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "ads111x.h"

#define LEDC_TIMER               (LEDC_TIMER_0)
#define LEDC_MODE                (LEDC_LOW_SPEED_MODE)
#define LEDC_FREQ_HZ             (50)
#define LEDC_DUTY_RESOLUTION     (LEDC_TIMER_14_BIT)

#define SERVO_PAN_GPIO           (16)
#define SERVO_PAN_MIN_PW_US      (500)
#define SERVO_PAN_MAX_PW_US      (2650)
#define SERVO_PAN_MIN_DEG        (0)
#define SERVO_PAN_MAX_DEG        (170)
#define SERVO_PAN_INVERSION      (true)

#define SERVO_TILT_GPIO          (17)
#define SERVO_TILT_MIN_PW_US     (850)
#define SERVO_TILT_MAX_PW_US     (2150)
#define SERVO_TILT_MIN_DEG       (0)
#define SERVO_TILT_MAX_DEG       (148)
#define SERVO_TILT_INVERSION     (true)

#define SERVO_PAN_START_DEG      (90)
#define SERVO_TILT_START_DEG     (45)

#define SHUNT_RESISTOR_MILLI_OHM (100)

#define I2C_GPIO_SCL             (9)
#define I2C_GPIO_SDA             (8)

#define I2C_PORT                 (0)
#define INA219_I2C_ADDR_1        (0x45)
#define INA219_I2C_ADDR_2        (0x40)

#define ADS1115_I2C_ADDR         (0x48)
#define ADS1115_GAIN             (ADS111X_GAIN_4V096)

#define LDR_TOP_LEFT             (ADS111X_MUX_2_GND)
#define LDR_TOP_RIGHT            (ADS111X_MUX_1_GND)
#define LDR_BOTTOM_LEFT          (ADS111X_MUX_3_GND)
#define LDR_BOTTOM_RIGHT         (ADS111X_MUX_0_GND)

#define TOP_LEFT                 (0)
#define TOP_RIGHT                (1)
#define BOTTOM_LEFT              (2)
#define BOTTOM_RIGHT             (3)

#define SERVO_PAN_CONFIG         { \
                                    .gpio_num = SERVO_PAN_GPIO, \
                                    .min_pulsewidth_us = SERVO_PAN_MIN_PW_US, \
                                    .max_pulsewidth_us = SERVO_PAN_MAX_PW_US, \
                                    .min_angle = SERVO_PAN_MIN_DEG, \
                                    .max_angle = SERVO_PAN_MAX_DEG, \
                                    .inverted = SERVO_PAN_INVERSION \
                                 }

#define SERVO_TILT_CONFIG        { \
                                    .gpio_num = SERVO_TILT_GPIO, \
                                    .min_pulsewidth_us = SERVO_TILT_MIN_PW_US, \
                                    .max_pulsewidth_us = SERVO_TILT_MAX_PW_US, \
                                    .min_angle = SERVO_TILT_MIN_DEG, \
                                    .max_angle = SERVO_TILT_MAX_DEG, \
                                    .inverted = SERVO_TILT_INVERSION \
                                 }

#define LEDC_TIMER_CONFIG        { \
                                    .duty_resolution = LEDC_DUTY_RESOLUTION, \
                                    .freq_hz = LEDC_FREQ_HZ, \
                                    .speed_mode = LEDC_MODE, \
                                    .timer_num = LEDC_TIMER, \
                                    .clk_cfg = LEDC_AUTO_CLK, \
                                 }

void servo_init(void);
void set_servo_position(int servo_num, int angle);
void servo_control_task(void *pvParameter);

#endif //ELT_SERVO_CONTROL_H