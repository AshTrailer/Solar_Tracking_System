#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

// ===== General macros =====
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

// ===== Motor pins =====
#define MOTOR_PIN_IN1  18
#define MOTOR_PIN_IN2  19
#define MOTOR_PIN_IN3  21
#define MOTOR_PIN_IN4  22

// ===== Motor parameters =====
#define MOTOR_STEPS_PER_REV 2048
#define MOTOR_MAX_ANGLE_DEG 360.0
#define MOTOR_MAX_SPEED_DEG_PER_SEC 20.0
#define MOTOR_MAX_ACCEL_DEG_PER_SEC2 15.0

// Step sequence
#define MOTOR_STEP_SEQUENCE_COUNT 8
#define MOTOR_STEPS_PER_CYCLE MOTOR_STEP_SEQUENCE_COUNT

// Control options
#define MOTOR_ENABLE_SHORTEST_PATH true
#define MOTOR_ENABLE_ANGLE_WRAP false
#define MOTOR_TASK_DELAY_MS 10
#define MOTOR_LOG_INTERVAL_MS 1000

// ===== Speed profile types =====
typedef enum {
    MOTOR_SPEED_TRAPEZOIDAL = 0,
    MOTOR_SPEED_S_CURVE = 1,
    MOTOR_SPEED_CONSTANT = 2
} motor_speed_profile_t;

// ===== Timing control =====
typedef struct {
    int64_t last_update_us;
    int64_t last_log_us;
    float dt;
    uint32_t task_run_count;
} motor_timing_t;

// ===== Speed planning =====
typedef struct {
    motor_speed_profile_t profile;
    float current_speed;
    float target_speed;
    float max_speed;
    float acceleration;
    float deceleration;
    float deceleration_distance;
    int direction;
    bool is_decelerating;
} motor_speed_planner_t;

// ===== Main motor control structure =====
typedef struct {
    // Pins
    int pin1, pin2, pin3, pin4;

    // Step control
    long current_step;
    long target_step;
    long shortest_path_step;

    // Angle control
    float current_angle;
    float target_angle;
    float default_angle;

    // Flags
    bool enable_shortest_path;
    bool enable_angle_wrap;
    bool is_moving;
    bool new_target_received;

    // Subsystems
    motor_timing_t timing;
    motor_speed_planner_t speed;

} StepperControl;

// Global motor instance and spinlock
extern StepperControl motor;
extern portMUX_TYPE motor_spinlock;

// ===== Function declarations =====

// Initialization
void stepper_init(StepperControl* m, int p1, int p2, int p3, int p4,
                  float maxSpeedDegPerSec, float maxAccelDegPerSec2,
                  motor_speed_profile_t speed_profile, float default_angle);

// Angle control
void stepper_move_to_angle(StepperControl* m, float angle);
void stepper_set_default_angle(StepperControl* m, float angle);
void stepper_return_to_default(StepperControl* m);
float stepper_get_current_angle(StepperControl* m);
void stepper_set_current_angle(StepperControl* m, float angle);

// Motion control
void stepper_stop(StepperControl* m);
bool stepper_is_moving(StepperControl* m);

// Configuration
void stepper_enable_shortest_path(StepperControl* m, bool enable);
void stepper_enable_angle_wrap(StepperControl* m, bool enable);
void stepper_set_speed_profile(StepperControl* m, motor_speed_profile_t profile);
void stepper_set_deceleration(StepperControl* m, float deceleration);

// Core operation
void stepper_run(StepperControl* m);

// Debug and monitoring
void stepper_print_performance(StepperControl* m);

#endif // STEPPER_MOTOR_H