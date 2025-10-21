#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "esp_system.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"

#include "cJSON.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_gap.h"
#include "nimble/ble.h"
#include "host/ble_uuid.h"

#include "os/os_mbuf.h"

#include "driver/ledc.h"
#include "esp_timer.h"

static portMUX_TYPE motor_spinlock = portMUX_INITIALIZER_UNLOCKED;

#define DEVICE_NAME "ESP32C6"
#define BLE_NUS_SERVICE_UUID 0x1A70
#define BLE_NUS_RX_CHAR_UUID 0x2B78
#define BLE_NUS_TX_CHAR_UUID 0x4C9A
static const char *TAG_BLE = "BLE";
static bool ble_ready = false;
static int connection_handle = -1;
static uint16_t tx_char_handle  = 0;
static uint16_t rx_char_handle = 0;
static bool client_subscribed = false;
static bool custom_service_registered = false;

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_SSID "S24"
#define WIFI_PASS "12746088"

#define STEPS_PER_REV 4096
#define MAX_SPEED_DEG_PER_SEC 10.0
#define MAX_ACCEL_DEG_PER_SEC2 5.0 
#define MAX_ANGLE_DEG 360.0

#define IN1  18
#define IN2  19
#define IN3  21
#define IN4  22
#define STEP_COUNT 8
#define MOTOR_LOG_INTERVAL_MS 1000

#define SERVO_PIN      23
#define SERVO_MAX_ANGLE 180
#define SERVO_UPDATE_MS 20
#define SERVO_MAX_SPEED_DEG_PER_SEC 10.0f
#define SERVO_SAFE_ANGLE 90.0f

static const char *TAG_STELLARIUM  = "stellarium";
const char *api_ips[] = {
    "10.183.59.199:8090" 
};
static const int api_poll_ms = 300000;
static EventGroupHandle_t s_wifi_event_group;

static int64_t g_last_time_update_ms = 0;
static time_t g_current_time = 0;

typedef struct {
    double latitude;
    double longitude;
    double altitude;
    char local_time[64];
    char utc_time[64];
    char time_zone[32];
} ObserverInfo;

typedef struct {
    bool above_horizon;
    double azimuth_deg;
    double altitude_deg;
} SunInfo;

static ObserverInfo g_observer;
static bool g_observer_valid = false;
static SunInfo g_sun;
static bool g_sun_valid = false;
static SemaphoreHandle_t observer_mutex = NULL;

static const int step_sequence[STEP_COUNT][4] = {
    {1,0,0,1},
    {0,0,0,1},
    {0,0,1,1},
    {0,0,1,0},
    {0,1,1,0},
    {0,1,0,0},
    {1,1,0,0},
    {1,0,0,0}
};

typedef struct {
    int pin1, pin2, pin3, pin4;
    long current_step;
    long target_step;
    float speed;
    float max_speed; 
    float accel;
    int direction;

    float current_angle;
    float target_angle;
    float default_angle;

    int64_t last_update_us;
} StepperControl;

typedef enum {
    MODE_SAFE = -1,
    MODE_WIFI_ONLINE,
    MODE_BLE_GPS_OFFLINE,
    MODE_MANUAL_OFFLINE
} ModeStatus;

typedef struct {
    int pin;
    float current_angle;
    float target_angle;
    float max_speed;
    int64_t last_update_us;
} ServoControl;

static ModeStatus g_mode_status = MODE_SAFE;
static bool wifi_connected = false;
static bool ble_gps_valid = false;
static bool manual_control_active = false;

static StepperControl motor;
static ServoControl servo;

static void start_advertising(void);
void set_servo_angle(float angle);
void manual_control_servo(void);
void wifi_auto_control_stepper_servo(void);
void offline_compute_stepper_servo(void);
void control_servo_fallback(void);
void servo_init(ServoControl *s, int pin, float max_speed);
void servo_update(ServoControl *s);
void stepper_init(StepperControl* m, int p1,int p2,int p3,int p4,
                  float maxSpeedDegPerSec, float maxAccelDegPerSec2);
void stepper_move_to_angle(StepperControl* m, float angle);
void stepper_set_default_angle(StepperControl* m, float angle);
void stepper_return_to_default(StepperControl* m);
float stepper_get_current_angle(StepperControl* m);
void stepper_set_current_angle(StepperControl* m, float angle);
void stepper_run(StepperControl* m);
void motor_task(void *arg);
void control_loop_task(void *arg);
void ble_init(void);
void ble_send_system_status(void);
void ble_send_data_format(void);
void update_system_time_from_utc(const char* utc_time_str);
time_t get_current_time(void);

// ========== 时间管理函数 ==========
void update_system_time_from_utc(const char* utc_time_str) {
    // 解析UTC时间字符串，格式如："2025-10-11T13:34:09.623"
    struct tm timeinfo = {0};
    int milliseconds = 0;
    
    // 解析ISO 8601格式
    if (sscanf(utc_time_str, "%d-%d-%dT%d:%d:%d.%d",
               &timeinfo.tm_year, &timeinfo.tm_mon, &timeinfo.tm_mday,
               &timeinfo.tm_hour, &timeinfo.tm_min, &timeinfo.tm_sec,
               &milliseconds) == 7) {
        
        timeinfo.tm_year -= 1900;  // tm_year是从1900开始
        timeinfo.tm_mon -= 1;      // tm_mon是0-11
        
        // 设置时区为UTC
        setenv("TZ", "UTC0", 1);
        tzset();
        
        g_current_time = mktime(&timeinfo);
        g_last_time_update_ms = esp_timer_get_time() / 1000;
        
        ESP_LOGI("TIME", "System time updated from UTC: %04d-%02d-%02d %02d:%02d:%02d.%03d",
                 timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, milliseconds);
    } else {
        // 尝试不带毫秒的格式
        if (sscanf(utc_time_str, "%d-%d-%dT%d:%d:%d",
                   &timeinfo.tm_year, &timeinfo.tm_mon, &timeinfo.tm_mday,
                   &timeinfo.tm_hour, &timeinfo.tm_min, &timeinfo.tm_sec) == 6) {
            
            timeinfo.tm_year -= 1900;
            timeinfo.tm_mon -= 1;
            
            setenv("TZ", "UTC0", 1);
            tzset();
            
            g_current_time = mktime(&timeinfo);
            g_last_time_update_ms = esp_timer_get_time() / 1000;
            
            ESP_LOGI("TIME", "System time updated from UTC: %04d-%02d-%02d %02d:%02d:%02d",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        } else {
            ESP_LOGE("TIME", "Failed to parse UTC time string: %s", utc_time_str);
        }
    }
}

time_t get_current_time(void) {
    if (g_current_time == 0) return 0;
    
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t elapsed_ms = now_ms - g_last_time_update_ms;
    
    return g_current_time + (elapsed_ms / 1000);
}

// ========== 模式管理 ==========
ModeStatus get_current_mode(void) {
    if (manual_control_active) return MODE_MANUAL_OFFLINE;
    if (wifi_connected) return MODE_WIFI_ONLINE;
    if (ble_gps_valid) return MODE_BLE_GPS_OFFLINE;
    return MODE_SAFE;
}

void update_mode_status(void) {
    ModeStatus new_mode = get_current_mode();
    if (new_mode != g_mode_status) {
        ESP_LOGI("MODE", "Mode changed: %d -> %d", g_mode_status, new_mode);
        g_mode_status = new_mode;
    }
}

// ========== 数据更新函数 ==========
bool ble_update_observer_info(const ObserverInfo* new_info, const SunInfo* sun_info, bool manual_mode) {
    if (wifi_connected && !manual_mode) {
        ESP_LOGW("BLE", "WiFi connected, BLE update rejected");
        return false;
    }

    if (observer_mutex && xSemaphoreTake(observer_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (new_info) {
            g_observer = *new_info;
            g_observer_valid = true;
            
            // 如果提供了UTC时间，更新系统时间
            if (strlen(new_info->utc_time) > 0) {
                update_system_time_from_utc(new_info->utc_time);
            }
            ESP_LOGI("BLE", "Observer updated via BLE");
        }

        if (manual_mode && sun_info != NULL) {
            g_sun = *sun_info;
            g_sun.above_horizon = (g_sun.altitude_deg > 0.0);
            g_sun_valid = true;
            ESP_LOGI("BLE", "Sun info updated via BLE manual mode");
        }

        if (manual_mode) {
            manual_control_active = true;
            ble_gps_valid = true;
        } else if (new_info) {
            manual_control_active = false;
            ble_gps_valid = true;
        }

        xSemaphoreGive(observer_mutex);
        return true;
    }
    return false;
}

bool wifi_update_observer_info(const ObserverInfo* new_info, const SunInfo* sun_info) {
    if (!wifi_connected) return false;

    if (observer_mutex && xSemaphoreTake(observer_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (new_info) {
            g_observer = *new_info;
            g_observer_valid = true;
            
            // 使用UTC时间更新系统时间
            if (strlen(new_info->utc_time) > 0) {
                update_system_time_from_utc(new_info->utc_time);
            }
        }

        if (sun_info != NULL) {
            g_sun = *sun_info;
            g_sun.above_horizon = (g_sun.altitude_deg > 0.0);
            g_sun_valid = true;
        }

        manual_control_active = false;
        ble_gps_valid = false;
        
        xSemaphoreGive(observer_mutex);
        ESP_LOGI("WIFI", "Observer and Sun info updated via WiFi");
        return true;
    }
    return false;
}

// ========== 控制函数 ==========
void control_servo_fallback(void) {
    set_servo_angle(SERVO_SAFE_ANGLE);
    stepper_return_to_default(&motor);
    ESP_LOGD("CONTROL", "Fallback mode - safe position");
}

void ble_send_system_status(void) {
    if (connection_handle == -1 || tx_char_handle == 0 || !client_subscribed) {
        return;
    }

    const char* mode_str;
    const char* wifi_status = wifi_connected ? "connected" : "disconnected";
    const char* ble_mode = manual_control_active ? "manual" : "gps";
    
    switch(g_mode_status) {
        case MODE_WIFI_ONLINE:       mode_str = "wifi_online"; break;
        case MODE_BLE_GPS_OFFLINE:   mode_str = "ble_gps_offline"; break;
        case MODE_MANUAL_OFFLINE:    mode_str = "manual_offline"; break;
        case MODE_SAFE:
        default:                     mode_str = "safe"; break;
    }

    char buf[128];
    int len = snprintf(buf, sizeof(buf), 
        "STATUS:{\"servo_angle\":%.2f,\"stepper_angle\":%.2f,\"mode\":\"%s\",\"wifi\":\"%s\",\"ble_mode\":\"%s\"}", 
        servo.current_angle, motor.current_angle, mode_str, wifi_status, ble_mode);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (!om) return;

    ble_gatts_notify_custom(connection_handle, tx_char_handle, om);
    ESP_LOGD("BLE", "Status sent: %s", buf);
}

void ble_send_data_format(void) {
    if (connection_handle == -1 || tx_char_handle == 0 || !client_subscribed) {
        return;
    }

    char buf[256];
    int len = snprintf(buf, sizeof(buf), 
        "DATA_FORMAT:{\"gps_format\":\"lat:xx.xx,lon:yy.yy,alt:zz.zz\","
        "\"time_format\":\"YYYY-MM-DDThh:mm:ss.sss\","
        "\"sun_format\":\"sun_alt:aa.aa,sun_az:bb.bb\","
        "\"manual_cmd\":\"manual\"}");

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (!om) return;

    ble_gatts_notify_custom(connection_handle, tx_char_handle, om);
    ESP_LOGI(TAG_BLE, "Data format info sent");
}

void control_loop_task(void *arg) {
    ESP_LOGI("CONTROL", "Control loop task started");
    int64_t last_log_time = 0;
    int64_t last_servo_log = 0;
    
    while (1) {
        update_mode_status();

        switch (g_mode_status) {
            case MODE_WIFI_ONLINE:
                wifi_auto_control_stepper_servo();
                break;
            case MODE_BLE_GPS_OFFLINE:
                offline_compute_stepper_servo();
                break;
            case MODE_MANUAL_OFFLINE:
                manual_control_servo();
                break;
            case MODE_SAFE:
            default:
                control_servo_fallback();
                break;
        }

        servo_update(&servo);
        
        int64_t now = esp_timer_get_time() / 1000;
        
        if (now - last_log_time > 5000) {
            ESP_LOGI("CONTROL", "Mode: %d, Servo: %.1f->%.1f, Stepper: %.1f->%.1f", 
                    g_mode_status, 
                    servo.current_angle, servo.target_angle,
                    motor.current_angle, motor.target_angle);
            last_log_time = now;
        }
        
        if (now - last_servo_log > 1000) {
            if (fabsf(servo.target_angle - servo.current_angle) > 1.0f) {
                ESP_LOGI("SERVO", "Moving: %.1f -> %.1f (diff: %.1f)", 
                        servo.current_angle, servo.target_angle,
                        servo.target_angle - servo.current_angle);
            }
            last_servo_log = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void servo_init(ServoControl *s, int pin, float max_speed) {
    s->pin = pin;
    s->current_angle = SERVO_SAFE_ANGLE;
    s->target_angle = SERVO_SAFE_ANGLE;
    s->max_speed = max_speed;
    s->last_update_us = esp_timer_get_time();

    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,  // 8192 steps
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,  // 50Hz for servos
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE("SERVO", "LEDC timer config failed: %s", esp_err_to_name(ret));
    }

    ledc_channel_config_t ch_conf = {
        .gpio_num       = s->pin,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ch_conf);
    if (ret != ESP_OK) {
        ESP_LOGE("SERVO", "LEDC channel config failed: %s", esp_err_to_name(ret));
    }

    set_servo_angle(SERVO_SAFE_ANGLE);
    servo_update(s);
    
    ESP_LOGI("SERVO", "Servo initialized on pin %d, PWM freq: %dHz", pin, timer_conf.freq_hz);
    ESP_LOGI("SERVO", "Initial duty for %.1f°: %lu", SERVO_SAFE_ANGLE, 
             (uint32_t)((0.5f + (SERVO_SAFE_ANGLE / 180.0f) * 2.0f) / 20.0f * 8191));
}

void set_servo_angle(float angle) {
    servo.target_angle = angle;
    if(servo.target_angle < 0) servo.target_angle = 0;
    if(servo.target_angle > SERVO_MAX_ANGLE) servo.target_angle = SERVO_MAX_ANGLE;
}

void servo_update(ServoControl *s) {
    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - s->last_update_us) / 1000000.0f;
    if (dt <= 0) return;
    
    s->last_update_us = now_us;

    float diff = s->target_angle - s->current_angle;
    float max_step = s->max_speed * dt;

    if (fabsf(diff) > max_step) {
        s->current_angle += (diff > 0 ? max_step : -max_step);
    } else {
        s->current_angle = s->target_angle;
    }

    int angle_int = (int)(s->current_angle + 0.5f);
    if(angle_int < 0) angle_int = 0;
    if(angle_int > SERVO_MAX_ANGLE) angle_int = SERVO_MAX_ANGLE;

    float duty_ms = 0.5f + ((float)angle_int / 180.0f) * 2.0f;
    uint32_t duty = (uint32_t)((duty_ms / 20.0f) * ((1 << 13) - 1));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    static int64_t last_servo_log = 0;
    static float last_logged_angle = -1.0f;
    
    int64_t now_ms = now_us / 1000;
    float angle_diff = fabsf(s->current_angle - last_logged_angle);
    
    if (now_ms - last_servo_log > 5000 && angle_diff > 2.0f) {
        ESP_LOGI("SERVO", "Servo angle: %.1f, duty: %lu", s->current_angle, duty);
        last_servo_log = now_ms;
        last_logged_angle = s->current_angle;
    }
}

static long deg_to_steps(float deg) {
    return (long)((deg / 360.0f) * STEPS_PER_REV);
}

static float steps_to_deg(long steps) {
    return (steps * 360.0f) / STEPS_PER_REV;
}

static void set_pins(StepperControl* m, int seq_index) {
    gpio_set_level(m->pin1, step_sequence[seq_index][0]);
    gpio_set_level(m->pin2, step_sequence[seq_index][1]); 
    gpio_set_level(m->pin3, step_sequence[seq_index][2]);
    gpio_set_level(m->pin4, step_sequence[seq_index][3]);
    
    static int step_count = 0;
    if (step_count++ % 100 == 0) {
        ESP_LOGD("STEPPER_PINS", "Step %d: IN1=%d, IN2=%d, IN3=%d, IN4=%d", 
                seq_index,
                step_sequence[seq_index][0],
                step_sequence[seq_index][1], 
                step_sequence[seq_index][2],
                step_sequence[seq_index][3]);
    }
}

void stepper_init(StepperControl* m, int p1,int p2,int p3,int p4,
                  float maxSpeedDegPerSec, float maxAccelDegPerSec2)
{
    m->pin1 = p1; m->pin2 = p2; m->pin3 = p3; m->pin4 = p4;
    m->current_step = 0;
    m->target_step = 0;
    m->speed = 0;
    m->direction = 1;
    m->last_update_us = esp_timer_get_time();

    m->max_speed = (maxSpeedDegPerSec / 360.0f) * STEPS_PER_REV;
    m->accel     = (maxAccelDegPerSec2 / 360.0f) * STEPS_PER_REV;

    m->current_angle = 0.0f;
    m->target_angle  = 0.0f;
    m->default_angle = 0.0f;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = 0;
    
    io_conf.pin_bit_mask |= (1ULL << p1);
    io_conf.pin_bit_mask |= (1ULL << p2); 
    io_conf.pin_bit_mask |= (1ULL << p3);
    io_conf.pin_bit_mask |= (1ULL << p4);
    
    gpio_config(&io_conf);
    
    gpio_set_level(p1, 0);
    gpio_set_level(p2, 0);
    gpio_set_level(p3, 0);
    gpio_set_level(p4, 0);
    
    set_pins(m, 0);
    
    ESP_LOGI("STEPPER", "Stepper initialized on pins %d,%d,%d,%d", p1, p2, p3, p4);
    ESP_LOGI("STEPPER", "Pin bitmask: 0x%08llx", io_conf.pin_bit_mask);
}

void stepper_move_to_angle(StepperControl* m, float angle) {
    if(angle < 0) angle = 0;
    if(angle > MAX_ANGLE_DEG) angle = MAX_ANGLE_DEG;
    m->target_angle = angle;
    m->target_step = deg_to_steps(angle);
}

void stepper_set_default_angle(StepperControl* m, float angle) {
    if(angle < 0) angle = 0;
    if(angle > MAX_ANGLE_DEG) angle = MAX_ANGLE_DEG;
    m->default_angle = angle;
}

void stepper_return_to_default(StepperControl* m) {
    stepper_move_to_angle(m, m->default_angle);
}

float stepper_get_current_angle(StepperControl* m) {
    return m->current_angle;
}

void stepper_set_current_angle(StepperControl* m, float angle) {
    if(angle < 0) angle = 0;
    if(angle > MAX_ANGLE_DEG) angle = MAX_ANGLE_DEG;
    m->current_angle = angle;
    m->target_angle  = angle;
    m->current_step  = deg_to_steps(angle);
    m->target_step   = deg_to_steps(angle);
}

void stepper_run(StepperControl* m) {
    long distance;

    portENTER_CRITICAL(&motor_spinlock);
    distance = m->target_step - m->current_step;
    portEXIT_CRITICAL(&motor_spinlock);

    if(distance == 0 && m->speed == 0) return;

    int target_direction = (distance > 0) ? 1 : -1;

    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - m->last_update_us) / 1000000.0f;
    if(dt <= 0) dt = 0.0001f;

    if(m->direction != target_direction && m->speed != 0) {
        if(m->speed > 0) {
            m->speed -= m->accel * dt;
            if(m->speed < 0) m->speed = 0;
        } else {
            m->speed += m->accel * dt;
            if(m->speed > 0) m->speed = 0;
        }
    } else {
        m->direction = target_direction;

        float f_distance = (float)distance;
        float decel_distance = (m->speed * m->speed) / (2.0f * m->accel);
        float desired_speed;

        if(fabsf(f_distance) < decel_distance) {
            desired_speed = sqrtf(2.0f * m->accel * fabsf(f_distance));
        } else {
            desired_speed = m->max_speed;
        }

        desired_speed *= m->direction;

        if(m->speed < desired_speed) {
            m->speed += m->accel * dt;
            if(m->speed > desired_speed) m->speed = desired_speed;
        } else if(m->speed > desired_speed) {
            m->speed -= m->accel * dt;
            if(m->speed < desired_speed) m->speed = desired_speed;
        }
    }

    if(m->speed != 0) {
        float step_interval_s = 1.0f / fabsf(m->speed);
        #define MIN_STEP_INTERVAL_US 10000
        if(step_interval_s * 1000000.0f < MIN_STEP_INTERVAL_US) {
            step_interval_s = MIN_STEP_INTERVAL_US / 1000000.0f;
        }
        
        if((now_us - m->last_update_us) >= step_interval_s * 1000000.0f) {
            m->current_step += m->direction;
            int seq_index = (m->current_step % STEP_COUNT + STEP_COUNT) % STEP_COUNT;
            set_pins(m, seq_index);
            m->last_update_us = now_us;
            m->current_angle = steps_to_deg(m->current_step);
        }
    }
}

void motor_task(void *arg) {
    ESP_LOGI("MOTOR", "Motor task started");
    
    while (!ble_ready) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI("MOTOR", "Motor task running");
    int64_t last_log = 0;

    while (1) {
        stepper_run(&motor);
        
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_log > MOTOR_LOG_INTERVAL_MS) {
            
            ESP_LOGI("MOTOR", "Stepper: current=%.1f°, target=%.1f°, steps=%ld, speed=%.1f", 
                    motor.current_angle, motor.target_angle, motor.current_step, motor.speed);
            last_log = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void wifi_auto_control_stepper_servo(void) {
    if (!g_observer_valid || !g_sun_valid) {
        ESP_LOGW("WIFI_CONTROL", "No valid observer or sun data, using fallback");
        control_servo_fallback();
        return;
    }
    
    float servo_target_angle = g_sun.altitude_deg;
    if (servo_target_angle < 0) servo_target_angle = 0;
    if (servo_target_angle > SERVO_MAX_ANGLE) servo_target_angle = SERVO_MAX_ANGLE;
    
    set_servo_angle(servo_target_angle);
    
    float stepper_target_angle = g_sun.azimuth_deg;
    stepper_move_to_angle(&motor, stepper_target_angle);
    
    ESP_LOGD("WIFI_CONTROL", "WiFi control: servo=%.1f°, stepper=%.1f°", 
             servo_target_angle, stepper_target_angle);
}

void offline_compute_stepper_servo(void) {
    if (!g_observer_valid) {
        ESP_LOGW("OFFLINE_CONTROL", "No valid observer data, using fallback");
        control_servo_fallback();
        return;
    }
    
    // 这里应该使用离线天文计算库计算太阳位置
    // 暂时使用简化逻辑 - 根据时间估算太阳位置
    time_t current_time = get_current_time();
    if (current_time == 0) {
        ESP_LOGW("OFFLINE_CONTROL", "No valid time data, using fallback");
        control_servo_fallback();
        return;
    }
    
    // 简化计算：假设正午太阳最高，日出/日落时最低
    struct tm *timeinfo = localtime(&current_time);
    int hour = timeinfo->tm_hour;
    
    // 简化模型 - 实际需要复杂的天文计算
    float estimated_altitude = 0.0f;
    if (hour >= 6 && hour <= 18) {
        estimated_altitude = 45.0f + (hour - 12) * 10.0f; // 简化估算
        if (estimated_altitude < 0) estimated_altitude = 0;
        if (estimated_altitude > 90) estimated_altitude = 90;
    }
    
    float estimated_azimuth = (hour - 6) * 15.0f; // 简化估算
    
    set_servo_angle(estimated_altitude);
    stepper_move_to_angle(&motor, estimated_azimuth);
    
    ESP_LOGD("OFFLINE_CONTROL", "Offline control: servo=%.1f°, stepper=%.1f°", 
             estimated_altitude, estimated_azimuth);
}

void manual_control_servo(void) {
    // 手动控制模式 - 暂时保持安全角度
    // 实际应该根据BLE接收的指令控制
    set_servo_angle(SERVO_SAFE_ANGLE);
    stepper_return_to_default(&motor);
    ESP_LOGD("MANUAL_CONTROL", "Manual control active - safe position");
}

// ========== BLE相关函数 ==========
static int gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, 
        uint16_t max_len, void *dst, uint16_t *len) {
    uint16_t om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    int rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

static int characteristic_write_cb(uint16_t conn_handle, uint16_t attr_handle, 
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    char buf[128] = {0};
    uint16_t len = sizeof(buf) - 1;
    int rc = gatt_svr_chr_write(ctxt->om, 1, sizeof(buf) - 1, buf, &len);
    if (rc != 0) return rc;
    buf[len] = '\0';

    ESP_LOGI(TAG_BLE, "BLE received: %s", buf);

    // 解析控制命令
    bool manual_mode = false;
    ObserverInfo obs = {0};
    SunInfo sun = {0};
    bool obs_valid = false;
    bool sun_valid = false;

    if (strstr(buf, "manual") != NULL) {
        manual_mode = true;
        ESP_LOGI(TAG_BLE, "BLE manual mode activated");
    }

    char *p = strstr(buf, "lat:");
    if (p) {
        obs.latitude = atof(p + 4);
        obs_valid = true;
    }
    
    p = strstr(buf, "lon:");
    if (p) {
        obs.longitude = atof(p + 4);
        obs_valid = true;
    }

    p = strstr(buf, "alt:");
    if (p) {
        obs.altitude = atof(p + 4);
        obs_valid = true;
    }

    p = strstr(buf, "time:");
    if (p) {
        char time_str[64] = {0};
        sscanf(p + 5, "%63[^,]", time_str);
        update_system_time_from_utc(time_str);
    }

    p = strstr(buf, "sun_alt:");
    if (p) {
        sun.altitude_deg = atof(p + 8);
        sun_valid = true;
    }
    
    p = strstr(buf, "sun_az:");
    if (p) {
        sun.azimuth_deg = atof(p + 7);
        sun_valid = true;
    }

    if (obs_valid || sun_valid) {
        ble_update_observer_info(obs_valid ? &obs : NULL, 
                               sun_valid ? &sun : NULL, manual_mode);
        ESP_LOGI(TAG_BLE, "BLE data updated: lat=%.6f lon=%.6f sun_alt=%.2f manual=%d", 
                 obs.latitude, obs.longitude, sun.altitude_deg, manual_mode);
    }

    ble_send_system_status();

    return 0;
}

static int characteristic_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "stepper:%.2f", motor.current_angle);
        int rc = os_mbuf_append(ctxt->om, buf, len);
        if (rc != 0) {
            ESP_LOGE(TAG_BLE, "Failed to append data to mbuf: %d", rc);
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return 0;
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1A70),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(BLE_NUS_RX_CHAR_UUID),
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .access_cb = characteristic_write_cb,
            },
            {
                .uuid = BLE_UUID16_DECLARE(BLE_NUS_TX_CHAR_UUID),
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .access_cb = characteristic_read_cb,
            },
            { 0 }
        }
    },
    { 0 }
};

static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC: {
            ESP_LOGI(TAG_BLE, "Registering SERVICE: %s, handle: %d", 
                     ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                     ctxt->svc.handle);
            
            uint16_t uuid_val = ble_uuid_u16(ctxt->svc.svc_def->uuid); 
            if (uuid_val == 0x1A70) {
                ESP_LOGI(TAG_BLE, "Custom Service 0x%04X Registered. Handle: %d", 
                         0x1A70, ctxt->svc.handle);
                custom_service_registered = true;
            }
            break;
        }

        case BLE_GATT_REGISTER_OP_CHR: {
            ESP_LOGI(TAG_BLE, "Registering CHARACTERISTIC: %s, val_handle: %d", 
                     ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                     ctxt->chr.val_handle);
            
            uint16_t chr_uuid_val = ble_uuid_u16(ctxt->chr.chr_def->uuid);
            
            if (chr_uuid_val == BLE_NUS_TX_CHAR_UUID) {
                tx_char_handle = ctxt->chr.val_handle;
                ESP_LOGI(TAG_BLE, "TX Characteristic Registered! Handle: %d", tx_char_handle);
            } else if (chr_uuid_val == BLE_NUS_RX_CHAR_UUID) {
                rx_char_handle = ctxt->chr.val_handle;
                ESP_LOGI(TAG_BLE, "RX Characteristic Registered! Handle: %d", rx_char_handle);
            }
            break;
        }

        case BLE_GATT_REGISTER_OP_DSC:
            ESP_LOGI(TAG_BLE, "Registering DESCRIPTOR: handle %d", ctxt->dsc.handle);
            break;

        default:
            break;
    }
}

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG_BLE, "Device connected, conn_handle: %d", event->connect.conn_handle);
                connection_handle = event->connect.conn_handle;
                
                // 连接后立即发送一次系统状态
                vTaskDelay(pdMS_TO_TICKS(100));
                ble_send_system_status();
                ble_send_data_format();
            } else {
                ESP_LOGE(TAG_BLE, "Connection failed, status: %d", event->connect.status);
                vTaskDelay(pdMS_TO_TICKS(1000));
                start_advertising();
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG_BLE, "Device disconnected, reason: %d", event->disconnect.reason);
            connection_handle = -1;
            client_subscribed = false;
            start_advertising();
            break;
            
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG_BLE, "Advertising complete");
            if (connection_handle == -1) {
                start_advertising();
            }
            break;
            
        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG_BLE, 
                "Subscribe event: conn_handle=%d, attr_handle=%d, reason=%d, prev_notify=%d, cur_notify=%d",
                event->subscribe.conn_handle,
                event->subscribe.attr_handle,
                event->subscribe.reason,
                event->subscribe.prev_notify,
                event->subscribe.cur_notify);
            
            if (event->subscribe.attr_handle == tx_char_handle || 
                event->subscribe.attr_handle == tx_char_handle + 1) {
                
                bool old_subscribed = client_subscribed;
                client_subscribed = event->subscribe.cur_notify;
                
                if (client_subscribed && !old_subscribed) {
                    ESP_LOGI(TAG_BLE, "Client SUBSCRIBED to TX characteristic");
                    // 订阅后立即发送状态
                    ble_send_system_status();
                } else if (!client_subscribed && old_subscribed) {
                    ESP_LOGI(TAG_BLE, "Client UNSUBSCRIBED from TX characteristic");
                }
            }
            break;
            
        default:
            ESP_LOGD(TAG_BLE, "Unhandled GAP event: %d", event->type);
            break;
    }
    return 0;
}

static void start_advertising(void) {
    ESP_LOGI(TAG_BLE, "Starting advertising...");
    
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; 
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; 
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(100);   
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(200);  

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    
    static ble_uuid16_t service_uuid = BLE_UUID16_INIT(BLE_NUS_SERVICE_UUID);
    fields.uuids16 = &service_uuid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.tx_pwr_lvl_is_present = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "Failed to set advertising fields: %d", rc);
        return;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                          &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "Failed to start advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG_BLE, "Advertising started - Device: %s, Service: 0x%04X", 
            DEVICE_NAME, BLE_NUS_SERVICE_UUID);
}

static void ble_app_on_sync(void) {
    int rc;

    ESP_LOGI(TAG_BLE, "=== BLE STACK SYNCED ===");

    uint8_t addr_val[6];
    ble_hs_id_copy_addr(0, addr_val, NULL);
    ESP_LOGI(TAG_BLE, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x",
            addr_val[5], addr_val[4], addr_val[3], 
            addr_val[2], addr_val[1], addr_val[0]);

    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "Failed to set device name: %d", rc);
        return;
    }

    ESP_LOGI(TAG_BLE, "Adding custom services...");
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    ESP_LOGI(TAG_BLE, "Add services result: %d", rc);

    if (rc == 0) {
        ESP_LOGI(TAG_BLE, "Custom services added successfully");
        custom_service_registered = true;
        vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
        ESP_LOGE(TAG_BLE, "Failed to add custom services: %d", rc);
    }

    start_advertising();
    ble_ready = true;
    ESP_LOGI(TAG_BLE, "=== BLE INITIALIZATION COMPLETED ===");
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG_BLE, "BLE Host Task Started");
    nimble_port_run(); 
    nimble_port_freertos_deinit();
}

void ble_app_on_reset(int reason)
{
    ESP_LOGI(TAG_BLE, "Resetting BLE state; reason=%d", reason);
}

void ble_init(void)
{
    ESP_LOGI(TAG_BLE, "=== INITIALIZING NIMBLE ===");

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;

    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "Failed to init nimble port: %d", rc);
        return;
    }

    ble_gatts_reset();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    nimble_port_freertos_init(ble_host_task);
}

// ========== WiFi和HTTP解析函数 ==========
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG_STELLARIUM, "Disconnected. Trying to reconnect...");
        wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_STELLARIUM, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        wifi_connected = true;
    }
}

/* ---- utility: append incoming HTTP data to buffer safely ---- */
static esp_err_t append_http_data(char **buf, size_t *len, const char *data, size_t data_len)
{
    size_t new_len = *len + data_len;
    char *tmp = realloc(*buf, new_len + 1);
    if (!tmp) {
        ESP_LOGE(TAG_STELLARIUM , "realloc failed");
        free(*buf);
        *buf = NULL;
        *len = 0;
        return ESP_FAIL;
    }
    *buf = tmp;
    memcpy(*buf + *len, data, data_len);
    *len = new_len;
    (*buf)[*len] = '\0';
    return ESP_OK;
}

/* ---- parse observer JSON into global structure ---- */
static void parse_observer_json(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG_STELLARIUM , "JSON parse error in observer data");
        return;
    }

    cJSON *location = cJSON_GetObjectItem(root, "location");
    cJSON *time_info = cJSON_GetObjectItem(root, "time");

    if (location && time_info) {
        cJSON *lat_j = cJSON_GetObjectItem(location, "latitude");
        cJSON *lon_j = cJSON_GetObjectItem(location, "longitude");
        cJSON *alt_j = cJSON_GetObjectItem(location, "altitude");
        cJSON *local_j = cJSON_GetObjectItem(time_info, "local");
        cJSON *utc_j   = cJSON_GetObjectItem(time_info, "utc");
        cJSON *tz_j    = cJSON_GetObjectItem(time_info, "timeZone");

        if (lat_j && lon_j && alt_j && local_j && utc_j && tz_j) {
            ObserverInfo tmp;
            tmp.latitude  = lat_j->valuedouble;
            tmp.longitude = lon_j->valuedouble;
            tmp.altitude  = alt_j->valuedouble;
            strncpy(tmp.local_time, local_j->valuestring, sizeof(tmp.local_time)-1);
            tmp.local_time[sizeof(tmp.local_time)-1] = '\0';
            strncpy(tmp.utc_time, utc_j->valuestring, sizeof(tmp.utc_time)-1);
            tmp.utc_time[sizeof(tmp.utc_time)-1] = '\0';
            strncpy(tmp.time_zone, tz_j->valuestring, sizeof(tmp.time_zone)-1);
            tmp.time_zone[sizeof(tmp.time_zone)-1] = '\0';

            if (observer_mutex && xSemaphoreTake(observer_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                g_observer = tmp;
                g_observer_valid = true;
                
                // 使用UTC时间更新系统时间
                update_system_time_from_utc(tmp.utc_time);
                
                xSemaphoreGive(observer_mutex);
            }
            ESP_LOGI(TAG_STELLARIUM , "Observer updated: lat=%f lon=%f local=%s",
                     tmp.latitude, tmp.longitude, tmp.local_time);
        } else {
            ESP_LOGE(TAG_STELLARIUM , "Missing fields in observer JSON");
        }
    } else {
        ESP_LOGE(TAG_STELLARIUM , "Observer JSON missing location/time");
    }

    cJSON_Delete(root);
}

/* ---- parse Sun info HTML ---- */
static void parse_sun_html(const char *html)
{
    // 从HTML中解析方位角和高度角
    // 查找 "A/a:" 行，格式如: "A/a: +310°22'30.0"/+48°47'02.8""
    char *a_ptr = strstr(html, "A/a:");
    if (!a_ptr) {
        ESP_LOGE(TAG_STELLARIUM, "Failed to find A/a in sun HTML");
        return;
    }

    double az = 0, alt = 0;
    char az_str[32] = {0};
    char alt_str[32] = {0};

    // 解析方位角和高度角
    if (sscanf(a_ptr, "A/a: +%31[^/]/+%31[^\"]", az_str, alt_str) == 2) {
        // 将度分秒转换为十进制度
        int az_deg, az_min;
        double az_sec;
        if (sscanf(az_str, "%d°%d'%lf\"", &az_deg, &az_min, &az_sec) == 3) {
            az = az_deg + az_min/60.0 + az_sec/3600.0;
        }
        
        int alt_deg, alt_min;
        double alt_sec;
        if (sscanf(alt_str, "%d°%d'%lf\"", &alt_deg, &alt_min, &alt_sec) == 3) {
            alt = alt_deg + alt_min/60.0 + alt_sec/3600.0;
        }
        
        ESP_LOGI(TAG_STELLARIUM, "Parsed sun: az=%s -> %.6f°, alt=%s -> %.6f°", 
                 az_str, az, alt_str, alt);
    } else {
        ESP_LOGE(TAG_STELLARIUM, "Failed to parse A/a format");
        return;
    }

    if (observer_mutex && xSemaphoreTake(observer_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        g_sun.azimuth_deg = az;
        g_sun.altitude_deg = alt;
        g_sun.above_horizon = (alt > 0.0);
        g_sun_valid = true;
        xSemaphoreGive(observer_mutex);
        
        ESP_LOGI(TAG_STELLARIUM, "Sun info updated: azimuth=%.2f°, altitude=%.2f°, above_horizon=%s",
                 az, alt, g_sun.above_horizon ? "YES" : "NO");
    }
}

/* ---- HTTP event handler for observer JSON ---- */
static esp_err_t _http_event_handler_status(esp_http_client_event_t *evt)
{
    static char *buf = NULL;
    static size_t len = 0;

    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (evt->data_len > 0) append_http_data(&buf, &len, evt->data, evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            if (buf && len > 0) parse_observer_json(buf);
            free(buf);
            buf = NULL;
            len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            if (buf) { free(buf); buf = NULL; len = 0; }
            break;
        default:
            break;
    }
    return ESP_OK;
}

/* ---- HTTP event handler for Sun HTML ---- */
static esp_err_t _http_event_handler_sun(esp_http_client_event_t *evt)
{
    static char *buf = NULL;
    static size_t len = 0;

    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (evt->data_len > 0) append_http_data(&buf, &len, evt->data, evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            if (buf && len > 0) parse_sun_html(buf);
            free(buf);
            buf = NULL;
            len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            if (buf) { free(buf); buf = NULL; len = 0; }
            break;
        default:
            break;
    }
    return ESP_OK;
}

/* ---- fetch observer JSON ---- */
static void fetch_json_url(const char *url,
                           esp_err_t (*event_handler)(esp_http_client_event_t *evt))
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .event_handler = event_handler,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return;
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

/* ---- fetch Stellarium observer + Sun info ---- */
void fetch_stellarium_status(const char *host_port)
{
    char url[128];

    // observer JSON
    snprintf(url, sizeof(url), "http://%s/api/main/status", host_port);
    fetch_json_url(url, _http_event_handler_status);

    // Sun info
    snprintf(url, sizeof(url), "http://%s/api/objects/info?name=Sun", host_port);
    fetch_json_url(url, _http_event_handler_sun);
}

/* ---- API task ---- */
void api_task(void *pv)
{
    ESP_LOGI(TAG_STELLARIUM, "API task started, polling every 5 minutes");
    
    while (1) {
        for (size_t i = 0; i < ARRAY_SIZE(api_ips); i++) {
            ESP_LOGI(TAG_STELLARIUM, "Polling %s", api_ips[i]);
            fetch_stellarium_status(api_ips[i]);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(api_poll_ms));
    }
}

void observer_print_task(void *pv)
{
    while (1) {
        if (observer_mutex && xSemaphoreTake(observer_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            if (g_observer_valid) {
                ESP_LOGI(TAG_STELLARIUM, "=== Observer Info (cached) ===");
                ESP_LOGI(TAG_STELLARIUM, "Latitude:  %f", g_observer.latitude);
                ESP_LOGI(TAG_STELLARIUM, "Longitude: %f", g_observer.longitude);
                ESP_LOGI(TAG_STELLARIUM, "Altitude:  %f", g_observer.altitude);
                ESP_LOGI(TAG_STELLARIUM, "Local Time: %s", g_observer.local_time);
                ESP_LOGI(TAG_STELLARIUM, "UTC Time:   %s", g_observer.utc_time);
                ESP_LOGI(TAG_STELLARIUM, "Time Zone:  %s", g_observer.time_zone);
            } else {
                ESP_LOGI(TAG_STELLARIUM, "Observer data not yet available");
            }

            if (g_sun_valid) {
                ESP_LOGI(TAG_STELLARIUM, "=== Sun Info (cached) ===");
                ESP_LOGI(TAG_STELLARIUM, "Above Horizon: %s", g_sun.above_horizon ? "YES" : "NO");
                ESP_LOGI(TAG_STELLARIUM, "Azimuth:  %.2f°", g_sun.azimuth_deg);
                ESP_LOGI(TAG_STELLARIUM, "Altitude: %.2f°", g_sun.altitude_deg);
            }

            xSemaphoreGive(observer_mutex);
        } else {
            ESP_LOGW(TAG_STELLARIUM, "Could not take observer_mutex to print");
        }

        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void wifi_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_wifi_event_group = xEventGroupCreate();
    ESP_LOGI(TAG_STELLARIUM, "Waiting for Wi-Fi connection...");
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_STELLARIUM, "Wi-Fi connected successfully!");
        wifi_connected = true;
    }
}

void app_main(void)
{
    ESP_LOGI("MAIN", "Application starting");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    observer_mutex = xSemaphoreCreateMutex();
    if (observer_mutex == NULL) {
        ESP_LOGE("MAIN", "Failed to create observer mutex");
        return;
    }

    stepper_init(&motor, IN1, IN2, IN3, IN4, MAX_SPEED_DEG_PER_SEC, MAX_ACCEL_DEG_PER_SEC2);
    stepper_set_current_angle(&motor, 0);
    stepper_set_default_angle(&motor, 0);
    
    servo_init(&servo, SERVO_PIN, SERVO_MAX_SPEED_DEG_PER_SEC);

    wifi_init();
    ble_init();

    vTaskDelay(pdMS_TO_TICKS(5000));
    
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_loop_task, "control_loop", 4096, NULL, 5, NULL);

    
    xTaskCreate(api_task, "api_task", 8192, NULL, 5, NULL);
    xTaskCreate(observer_print_task, "observer_prn", 4096, NULL, 4, NULL);

    ESP_LOGI("MAIN", "Application started successfully");
    ESP_LOGI("MAIN", "Current mode: %d", g_mode_status);
    ESP_LOGI("MAIN", "Waiting for BLE connection...");
}