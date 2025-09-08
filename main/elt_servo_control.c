#include "include/elt_servo_control.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "ina219.h"
#include <string.h>
#include <math.h>
#include "include/elt_shared_data.h"

//#include <inttypes.h>

static const char *TAG = "SERVO";

typedef struct {
    int gpio_num;
    uint32_t min_pulsewidth_us;
    uint32_t max_pulsewidth_us;
    int min_angle;
    int max_angle;
    bool inverted;
} servo_config_t;

typedef enum {
    SERVO_PAN = 0,
    SERVO_TILT
} servo_index_t;

static const int servo_pins[NUM_SERVOS] = {
    SERVO_PAN_GPIO,
    SERVO_TILT_GPIO 
};

static const servo_config_t servos[NUM_SERVOS] = {
        SERVO_PAN_CONFIG,
        SERVO_TILT_CONFIG
};

static const uint8_t addr = ADS1115_I2C_ADDR;
static i2c_dev_t device;
static float gain_val;

static float current_1 = 0;
static float est_1 = 0;

static float current_2 = 0;
static float est_2 = 0;

static float measure(void)
{
    float mea_voltage = 0;
    bool busy;
    if(ads111x_start_conversion(&device)!=ESP_OK) {
        ESP_LOGE(TAG, "Error starting conversion on device");
        return -1;
    };
    do{
        vTaskDelay(pdMS_TO_TICKS(1));
      ads111x_is_busy(&device, &busy);
    }
    while(busy);

    int16_t value = 0;
    if(ads111x_get_value(&device, &value) == ESP_OK){
        if (value < 0 ) value = 0;
        mea_voltage = ((float)value / (float)ADS111X_MAX_VALUE) * gain_val;
    }
    else
    {
        ESP_LOGE(TAG, "Error reading value from device ");
    }
    return mea_voltage;
}

static uint32_t angle_to_duty(int angle, int servo_num) {
    if (servo_num < 0 || servo_num >= NUM_SERVOS) {
        ESP_LOGE(TAG, "Invalid servo number: %d", servo_num);
        return 0;
    }

    const servo_config_t *servo = &servos[servo_num];

    if (angle < servo->min_angle) angle = servo->min_angle;
    if (angle > servo->max_angle) angle = servo->max_angle;

    if (servo->inverted) {
        angle = servo->max_angle - (angle - servo->min_angle);
    }

    uint32_t min_pw = servo->min_pulsewidth_us;
    uint32_t max_pw = servo->max_pulsewidth_us;

    uint32_t pulsewidth = min_pw + ((max_pw - min_pw)*(angle - servo->min_angle)) / (servo->max_angle - servo->min_angle);
    
    uint32_t duty = (pulsewidth * (1 << LEDC_DUTY_RESOLUTION)) / (1000000 / LEDC_FREQ_HZ);
    
    //ESP_LOGI(TAG, "Angle: %d, Duty: %" PRIu32, angle, duty);
    return duty;
}

void servo_init(void) {
    ledc_timer_config_t ledc_timer = LEDC_TIMER_CONFIG;
    ledc_timer_config(&ledc_timer);

    for(int i = 0; i<NUM_SERVOS; i++) {
        ledc_channel_config_t ledc_channel = {
            .channel = LEDC_CHANNEL_0 + i,
            .duty = 0,
            .gpio_num = servo_pins[i],
            .speed_mode = LEDC_MODE,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .hpoint = 0,
        };
        ledc_channel_config(&ledc_channel);
    }
}

void set_servo_position(int servo_num, int angle) {
    if(servo_num < 0 || servo_num>= NUM_SERVOS) {
        ESP_LOGE(TAG, "Invalid servo number: %d", servo_num);
        return;
    }
    const servo_config_t *servo = &servos[servo_num];

    if (angle < servo->min_angle ) angle = servo->min_angle;
    if (angle > servo->max_angle ) angle = servo->max_angle;

    if(xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if(current_servo_angles[servo_num] == angle) 
        {
            xSemaphoreGive(servo_mutex);
            return;    
        }
        //ESP_LOGI(TAG, "Setting servo %d to angle %d", servo_num, angle);
        uint32_t duty = angle_to_duty(angle, servo_num);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0 + servo_num, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0 + servo_num);
        current_servo_angles[servo_num] = angle;
        xSemaphoreGive(servo_mutex);
    }
    else {
        ESP_LOGE(TAG, "Failed to acquire current servo angle mutex");
    }
}

float read_ldr_voltage(int mux) {
    ESP_ERROR_CHECK(ads111x_set_input_mux(&device, mux));
    vTaskDelay(pdMS_TO_TICKS(10));
    return measure();
}

void servo_control_task(void *pvParameter) {    
    i2cdev_init();
    
    servo_init();
    
    vTaskDelay(pdMS_TO_TICKS(100));
    ina219_t dev1;
    memset(&dev1, 0, sizeof(ina219_t));
    ESP_ERROR_CHECK(ina219_init_desc(&dev1, INA219_I2C_ADDR_1, I2C_PORT, I2C_GPIO_SDA, I2C_GPIO_SCL));
    ESP_ERROR_CHECK(ina219_init(&dev1));
    ESP_ERROR_CHECK(ina219_configure(&dev1, INA219_BUS_RANGE_16V, INA219_GAIN_1, INA219_RES_12BIT_128S, INA219_RES_12BIT_128S, INA219_MODE_CONT_SHUNT));
    ESP_ERROR_CHECK(ina219_calibrate(&dev1, (float)SHUNT_RESISTOR_MILLI_OHM / 1000.0f));

    ina219_t dev2;
    memset(&dev2, 0, sizeof(ina219_t));
    ESP_ERROR_CHECK(ina219_init_desc(&dev2, INA219_I2C_ADDR_2, I2C_PORT, I2C_GPIO_SDA, I2C_GPIO_SCL));
    ESP_ERROR_CHECK(ina219_init(&dev2));
    ESP_ERROR_CHECK(ina219_configure(&dev2, INA219_BUS_RANGE_16V, INA219_GAIN_1, INA219_RES_12BIT_128S, INA219_RES_12BIT_128S, INA219_MODE_CONT_SHUNT));
    ESP_ERROR_CHECK(ina219_calibrate(&dev2, (float)SHUNT_RESISTOR_MILLI_OHM / 1000.0f));

    static int servo_0_pos = SERVO_PAN_START_DEG;
    static int servo_1_pos = SERVO_TILT_START_DEG;

    gain_val = ads111x_gain_values[ADS111X_GAIN_4V096];
    ESP_ERROR_CHECK(ads111x_init_desc(&device, addr, I2C_PORT, I2C_GPIO_SDA, I2C_GPIO_SCL));

    ESP_ERROR_CHECK(ads111x_set_mode(&device, ADS111X_MODE_SINGLE_SHOT));
    ESP_ERROR_CHECK(ads111x_set_data_rate(&device, ADS111X_DATA_RATE_64));
    ESP_ERROR_CHECK(ads111x_set_input_mux(&device, ADS111X_MUX_0_GND));
    ESP_ERROR_CHECK(ads111x_set_gain(&device, ADS111X_GAIN_4V096));
    while(1)
    {       
        float ldr[NUM_LDRS];
        ldr[TOP_LEFT] = read_ldr_voltage(LDR_TOP_LEFT);
        ldr[TOP_RIGHT] = read_ldr_voltage(LDR_TOP_RIGHT);
        ldr[BOTTOM_LEFT] = read_ldr_voltage(LDR_BOTTOM_LEFT);
        ldr[BOTTOM_RIGHT] = read_ldr_voltage(LDR_BOTTOM_RIGHT);
            
        if(xSemaphoreTake(ldr_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            for(int i = 0; i < NUM_LDRS; i++) {
                ldr_voltage[i] = ldr[i];
            }
            xSemaphoreGive(ldr_mutex);
        } else {
            ESP_LOGE(TAG, "Failed to acquire ldr voltage mutex");
        }

        //ESP_LOGI("LIGHT", "Top Left: %f, Top Right: %f, Bottom Left: %f, Bottom Right: %f", ldr[TOP_LEFT], ldr[TOP_RIGHT], ldr[BOTTOM_LEFT], ldr[BOTTOM_RIGHT]);
        float top = ldr[TOP_LEFT] + ldr[TOP_RIGHT];
        float bottom = ldr[BOTTOM_LEFT] + ldr[BOTTOM_RIGHT];
        float left = ldr[TOP_LEFT] + ldr[BOTTOM_LEFT];
        float right = ldr[TOP_RIGHT] + ldr[BOTTOM_RIGHT];

        //ESP_LOGI("LIGHT", "Top: %f, Bottom: %f, Left: %f, Right: %f", top, bottom, left, right);

        float light_total = left + right;

        //ESP_LOGI("LIGHT", "Total: %f", light_total);

        float horiz_diff = 0.0f;
        float vert_diff = 0.0f;

        if (light_total > 0.05f) {
            horiz_diff = (left-right) / light_total;
            vert_diff = (top-bottom) / light_total;
            //ESP_LOGI("LIGHT", "Horiz Diff: %f, Vert Diff: %f", horiz_diff, vert_diff);
            float brightness_scale = sqrtf(light_total/3.3f);
            //ESP_LOGI("LIGHT", "Brightness Scale: %f", brightness_scale);
            horiz_diff *= brightness_scale;
            vert_diff *= brightness_scale;
            //ESP_LOGI("LIGHT", "Scaled Horiz Diff: %f, Scaled Vert Diff: %f", horiz_diff, vert_diff);
        }

        #define DEADZONE 0.000005f
        #define SERVO_MAX_STEP 2

        int delta_x = fabs(horiz_diff) > DEADZONE ? (int)roundf(horiz_diff * SERVO_MAX_STEP) : 0;
        int delta_y = fabs(vert_diff) > DEADZONE ? (int)roundf(vert_diff * SERVO_MAX_STEP) : 0;
        //ESP_LOGI("LIGHT", "Delta X: %d, Delta Y: %d", delta_x, delta_y);
        bool tracking_copy = false;
        if (xSemaphoreTake(tracking_mutex,  pdMS_TO_TICKS(100)) == pdTRUE) {
            tracking_copy = tracking;
            xSemaphoreGive(tracking_mutex);
        } else {
            ESP_LOGE(TAG, "Failed to acquire tracking mutex");
        }
        if (tracking_copy) { 
            servo_0_pos = servo_0_pos - delta_x;
            set_servo_position(0, servo_0_pos);
            servo_1_pos = servo_1_pos + delta_y;
            set_servo_position(1, servo_1_pos);
            //ESP_LOGI("LIGHT", "Servo 0 Pos: %d, Servo 1 Pos: %d", servo_0_pos, servo_1_pos);
        }
        else {
            vTaskDelay(pdMS_TO_TICKS(500));
            if(xSemaphoreTake(servo_mutex, pdMS_TO_TICKS(100))==pdTRUE) {
                servo_0_pos = current_servo_angles[0];
                servo_1_pos = current_servo_angles[1];
                xSemaphoreGive(servo_mutex);
            }
            else {
                ESP_LOGE(TAG, "Failed to acquire servo mutex");
            }
        }
        //vTaskDelay(pdMS_TO_TICKS(3000));
        ESP_ERROR_CHECK(ina219_get_current(&dev1, &current_1));
        if (current_1 < 0.0f) {
            current_1 = 0;
        }
        est_1 = -11.2 * current_1 * current_1 + 7.3 * current_1;
        
        ESP_ERROR_CHECK(ina219_get_current(&dev2, &current_2));
        if (current_2 < 0.0f) {
            current_2 = 0;
        }
        est_2 = -11.2 * current_2 * current_2 + 7.3 * current_2;

        if (xSemaphoreTake(shunt_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            shunt_current_1 = current_1;
            shunt_current_2 = current_2;
            power_est_1 = est_1;
            power_est_2 = est_2;
            xSemaphoreGive(shunt_mutex);
        } else {
            ESP_LOGE(TAG, "Failed to acquire shunt current mutex");
        }
    }
}