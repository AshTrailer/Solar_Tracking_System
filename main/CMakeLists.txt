idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        bt
        cJSON
        stepper_motor
        nvs_flash
        esp_wifi
        esp_event
        esp_system
        esp_netif
        fatfs
        esp_eth
        esp_http_client
        esp_http_server
        esp_https_ota
        esp_https_server
        app_update
        esp_timer
)