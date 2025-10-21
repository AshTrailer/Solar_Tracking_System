idf_component_register(
    SRCS "stepper_motor.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver
        esp_timer
        freertos 
        esp_timer 
)