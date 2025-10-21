#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

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
#define MAX_SPEED_DEG_PER_SEC 20.0
#define MAX_ACCEL_DEG_PER_SEC2 15.0 
#define MAX_ANGLE_DEG 360.0

#define IN1  18
#define IN2  19
#define IN3  21
#define IN4  22
#define STEP_COUNT 8

static const int step_sequence[STEP_COUNT][4] = {
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1}
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

StepperControl motor;

static long deg_to_steps(float deg) {
    return (long)((deg / 360.0f) * STEPS_PER_REV);
}

static float steps_to_deg(long steps) {
    return (steps * 360.0f) / STEPS_PER_REV;
}

float parse_stepper_angle(const char* buf) {
    const char* p = strstr(buf, "stepper:");
    if(!p) return -1;
    p += strlen("stepper:");
    float angle = atof(p);
    if(angle < 0) angle = 0;
    if(angle > MAX_ANGLE_DEG) angle = MAX_ANGLE_DEG;
    return angle;
}

static void set_pins(StepperControl* m, int seq_index) {
    gpio_set_level(m->pin1, step_sequence[seq_index][0]);
    gpio_set_level(m->pin2, step_sequence[seq_index][1]);
    gpio_set_level(m->pin3, step_sequence[seq_index][2]);
    gpio_set_level(m->pin4, step_sequence[seq_index][3]);
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

    int pins[4] = {p1,p2,p3,p4};
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = 0;
    for(int i=0;i<4;i++) io_conf.pin_bit_mask |= (1ULL << pins[i]);
    gpio_config(&io_conf);
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

    m->direction = (distance > 0) ? 1 : -1;

    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - m->last_update_us) / 1000000.0f;
    if(dt <= 0) dt = 0.0001f;

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

    if(m->speed != 0) {
        float step_interval_s = 1.0f / fabsf(m->speed);
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

    while (1) {
        stepper_run(&motor);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static int gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, 
        uint16_t max_len, void *dst, uint16_t *len);
void ble_send_angle(float angle) ;
static int characteristic_write_cb(uint16_t conn_handle, uint16_t attr_handle, 
        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    
    ESP_LOGI(TAG_BLE, "=== CHARACTERISTIC WRITE CALLBACK CALLED ===");
    ESP_LOGI(TAG_BLE, "conn_handle: %d, attr_handle: %d, op: %d", 
             conn_handle, attr_handle, ctxt->op);

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char buf[32] = {0};
        uint16_t len = sizeof(buf) - 1;
        int rc = gatt_svr_chr_write(ctxt->om, 1, sizeof(buf) - 1, buf, &len);
        if (rc != 0) return rc;

        buf[len] = '\0';
        float angle = atof(buf);
        if (angle < 0) angle = 0;
        if (angle > MAX_ANGLE_DEG) angle = MAX_ANGLE_DEG;
        
        portENTER_CRITICAL(&motor_spinlock);
        motor.target_angle = angle;
        motor.target_step = deg_to_steps(angle);
        portEXIT_CRITICAL(&motor_spinlock);

        ESP_LOGI(TAG_BLE, "Received angle: %.2f", motor.target_angle);
        ble_send_angle(motor.target_angle);
    }
    return 0;
}

static int characteristic_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        float current_angle;
        
        portENTER_CRITICAL(&motor_spinlock);
        current_angle = motor.current_angle;
        portEXIT_CRITICAL(&motor_spinlock);
        
        char buf[16];
        int len = snprintf(buf, sizeof(buf), "%.2f", current_angle);
        int rc = os_mbuf_append(ctxt->om, buf, len);
        if (rc != 0) {
            ESP_LOGE(TAG_BLE, "Failed to append data to mbuf: %d", rc);
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        ESP_LOGI(TAG_BLE, "Characteristic read: %.2f", current_angle);
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

    ESP_LOGI(TAG_BLE, "=== GATT REGISTER CALLBACK CALLED ===");
    ESP_LOGI(TAG_BLE, "Operation type: %d", ctxt->op);

    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC: {
            ESP_LOGI(TAG_BLE, "Registering SERVICE: %s, handle: %d", 
                     ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                     ctxt->svc.handle);
            
            uint16_t uuid_val = ble_uuid_u16(ctxt->svc.svc_def->uuid); 
            ESP_LOGI(TAG_BLE, "Service UUID value: 0x%04X", uuid_val);
            
            if (uuid_val == 0x1A70) {
                ESP_LOGI(TAG_BLE, "SUCCESS: Custom Service 0x%04X Registered. Handle: %d", 
                         0x1A70, ctxt->svc.handle);
                custom_service_registered = true;
                ESP_LOGI(TAG_BLE, "=== CUSTOM SERVICE REGISTRATION CONFIRMED ===");
            } else {
                ESP_LOGI(TAG_BLE, "This is a standard service: 0x%04X", uuid_val);
            }
            break;
        }

        case BLE_GATT_REGISTER_OP_CHR: {
            ESP_LOGI(TAG_BLE, "Registering CHARACTERISTIC: %s, val_handle: %d", 
                     ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                     ctxt->chr.val_handle);
            
            uint16_t chr_uuid_val = ble_uuid_u16(ctxt->chr.chr_def->uuid);
            ESP_LOGI(TAG_BLE, "Characteristic UUID value: 0x%04X", chr_uuid_val);
            
            if (chr_uuid_val == BLE_NUS_TX_CHAR_UUID) {  // TX Characteristic
                tx_char_handle = ctxt->chr.val_handle;
                ESP_LOGI(TAG_BLE, "TX Characteristic Registered! Handle: %d", tx_char_handle);
            } else if (chr_uuid_val == BLE_NUS_RX_CHAR_UUID) {  // RX Characteristic
                rx_char_handle = ctxt->chr.val_handle;
                ESP_LOGI(TAG_BLE, "RX Characteristic Registered! Handle: %d", rx_char_handle);
            } else {
                ESP_LOGI(TAG_BLE, "This is a standard characteristic: 0x%04X", chr_uuid_val);
            }
            break;
        }

        case BLE_GATT_REGISTER_OP_DSC:
            ESP_LOGI(TAG_BLE, "Registering DESCRIPTOR: handle %d", ctxt->dsc.handle);
            break;

        default:
            ESP_LOGI(TAG_BLE, "Unknown register event: %d", ctxt->op);
            break;
    }

    ESP_LOGI(TAG_BLE, "=== GATT REGISTER CALLBACK COMPLETED ===");
}

static int gap_event_handler(struct ble_gap_event *event, void *arg);
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

    ESP_LOGI(TAG_BLE, "Setting advertising fields...");
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "Failed to set advertising fields: %d", rc);
        return;
    } else {
        ESP_LOGI(TAG_BLE, " Advertising fields set successfully");
    }

    ESP_LOGI(TAG_BLE, "Starting advertising...");
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                          &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "Failed to start advertising: %d", rc);
        return;
    } else {
        ESP_LOGI(TAG_BLE, "Advertising started successfully");
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

    ESP_LOGI(TAG_BLE, "Adding custom services after stack sync...");
    
    ESP_LOGI(TAG_BLE, "Service definition pointer: %p", gatt_svr_svcs);
    ESP_LOGI(TAG_BLE, "First service UUID: 0x%04X", 
             ble_uuid_u16(gatt_svr_svcs[0].uuid));
    
    /*
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    ESP_LOGI(TAG_BLE, "Add services result: %d", rc);

    if (rc == 0) {
        ESP_LOGI(TAG_BLE, "Custom services added successfully");
        custom_service_registered = true;
        
        ESP_LOGI(TAG_BLE, "Waiting for registration callbacks...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        if (custom_service_registered) {
            ESP_LOGI(TAG_BLE, "Custom service registration confirmed!");
        } else {
            ESP_LOGW(TAG_BLE, "Custom service registration callback not received");
        }
    } else {
        ESP_LOGE(TAG_BLE, "Failed to add custom services: %d", rc);
    }
    */

    ESP_LOGI(TAG_BLE, "Starting advertising...");
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
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG_BLE, "Adding custom services before host task start...");
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    ESP_LOGI(TAG_BLE, "Add services result: %d", rc);
}

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG_BLE, "Device connected, conn_handle: %d", event->connect.conn_handle);
                connection_handle = event->connect.conn_handle;
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

void ble_send_angle(float angle) {
    if (connection_handle == -1) {
        ESP_LOGD(TAG_BLE, "No connection, skipping angle send: %.2f", angle);
        return;
    }

    if (tx_char_handle == 0) {
        ESP_LOGE(TAG_BLE, "TX characteristic handle not available");
        return;
    }

    if (!client_subscribed) {
        ESP_LOGD(TAG_BLE, "Client not subscribed, skipping notification: %.2f", angle);
        return;
    }

    char buf[32];
    int len = snprintf(buf, sizeof(buf), "%.2f", angle);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    if (!om) {
        ESP_LOGE(TAG_BLE, "Failed to allocate mbuf for TX");
        return;
    }

    int rc = ble_gatts_notify_custom(connection_handle, tx_char_handle, om);
    
    if (rc != 0) {
        ESP_LOGE(TAG_BLE, "Notify failed: %d", rc);
        os_mbuf_free_chain(om);
    } else {
        ESP_LOGI(TAG_BLE, "Angle notification sent: %.2f", angle);
    }
}

void ble_send_string(const char* str) {
    if (connection_handle == -1 || !client_subscribed || tx_char_handle == 0) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(str, strlen(str));
    if (!om) return;

    int rc = ble_gatts_notify_custom(connection_handle, tx_char_handle, om);
    if (rc != 0) os_mbuf_free_chain(om);
}

/* ---- app_main: init wifi, create mutex and tasks ---- */
void app_main(void)
{
    ESP_LOGI("MAIN", "Application starting");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    stepper_init(&motor, IN1, IN2, IN3, IN4, MAX_SPEED_DEG_PER_SEC, MAX_ACCEL_DEG_PER_SEC2);
    stepper_set_current_angle(&motor, 0);
    
    ble_init();
    
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);

    ESP_LOGI("MAIN", "Application started successfully");    

}










