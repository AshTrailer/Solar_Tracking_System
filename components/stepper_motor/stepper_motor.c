#include "stepper_motor.h"

static const int motor_step_sequence[MOTOR_STEP_SEQUENCE_COUNT][4] = {
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1}
};

StepperControl motor;
portMUX_TYPE motor_spinlock = portMUX_INITIALIZER_UNLOCKED;

static long motor_deg_to_steps(float deg);
static float motor_steps_to_deg(long steps);
static void motor_set_pins(StepperControl* m, int seq_index);
static void motor_update_timing(StepperControl* m);
static void motor_calculate_shortest_path(StepperControl* m, float target_angle);
static void motor_trapezoidal_speed_planning(StepperControl* m);
static void motor_execute_step(StepperControl* m);
static void motor_log_status(StepperControl* m);

// Convert angle (degrees) to motor steps
static long motor_deg_to_steps(float deg) {
    return (long)((deg / MOTOR_MAX_ANGLE_DEG) * MOTOR_STEPS_PER_REV);
}

// Convert motor steps to angle (degrees)
static float motor_steps_to_deg(long steps) {
    return (steps * MOTOR_MAX_ANGLE_DEG) / MOTOR_STEPS_PER_REV;
}

// Set GPIO pins according to step sequence
static void motor_set_pins(StepperControl* m, int seq_index) {
    gpio_set_level(m->pin1, motor_step_sequence[seq_index][0]);
    gpio_set_level(m->pin2, motor_step_sequence[seq_index][1]);
    gpio_set_level(m->pin3, motor_step_sequence[seq_index][2]);
    gpio_set_level(m->pin4, motor_step_sequence[seq_index][3]);
}

// Update time delta and task run count
static void motor_update_timing(StepperControl* m) {
    int64_t now_us = esp_timer_get_time();
    m->timing.dt = (now_us - m->timing.last_update_us) / 1000000.0f;
    if (m->timing.dt <= 0) m->timing.dt = 0.0001f;
    m->timing.last_update_us = now_us;
    m->timing.task_run_count++;
}

// Calculate shortest rotation path to target angle
static void motor_calculate_shortest_path(StepperControl* m, float target_angle) {
    if (!m->enable_shortest_path) {
        m->target_step = motor_deg_to_steps(target_angle);
        return;
    }
    
    long target_step = motor_deg_to_steps(target_angle);
    long direct_distance = target_step - m->current_step;
    
    long clockwise_distance = direct_distance;
    long counterclockwise_distance = direct_distance;
    
    if (direct_distance > 0) {
        counterclockwise_distance = direct_distance - MOTOR_STEPS_PER_REV;
    } else {
        clockwise_distance = direct_distance + MOTOR_STEPS_PER_REV;
    }
    
    // Choose shorter distance
    if (abs(counterclockwise_distance) < abs(clockwise_distance)) {
        m->shortest_path_step = m->current_step + counterclockwise_distance;
    } else {
        m->shortest_path_step = m->current_step + clockwise_distance;
    }
    
    m->target_step = m->shortest_path_step;
    
    ESP_LOGI("MOTOR", "Shortest path: %.1f° -> %ld steps (direct: %ld steps)", 
             target_angle, m->target_step, direct_distance);
}

// Plan speed using trapezoidal profile
static void motor_trapezoidal_speed_planning(StepperControl* m) {
    long distance = m->target_step - m->current_step;
    
    // Stop if no distance and speed is zero
    if (distance == 0 && m->speed.current_speed == 0) {
        m->is_moving = false;
        m->speed.is_decelerating = false;
        return;
    }
    
    m->is_moving = true;
    int new_direction = (distance > 0) ? 1 : -1;
    
    // Check if direction needs changing
    bool direction_changed = (new_direction != m->speed.direction) && (m->speed.current_speed != 0);
    
    // Start deceleration if changing direction or new target received
    if ((direction_changed || m->new_target_received) && !m->speed.is_decelerating) {
        m->speed.is_decelerating = true;
        ESP_LOGI("MOTOR", "Starting deceleration: direction_changed=%d, new_target=%d", 
                direction_changed, m->new_target_received);
    }
    
    // Handle deceleration
    if (m->speed.is_decelerating) {
        float decel = m->speed.deceleration * m->timing.dt;
        
        if (fabsf(m->speed.current_speed) <= decel) {
            m->speed.current_speed = 0.0f;
            m->speed.is_decelerating = false;
            
            if (direction_changed) {
                m->speed.direction = new_direction;
                ESP_LOGI("MOTOR", "Direction change completed: %d -> %d", 
                        m->speed.direction, new_direction);
            }
            
            if (m->new_target_received) {
                m->new_target_received = false;
                ESP_LOGI("MOTOR", "New target processing completed");
            }
        } else {
            // Reduce speed gradually
            if (m->speed.current_speed > 0) {
                m->speed.current_speed -= decel;
            } else {
                m->speed.current_speed += decel;
            }
        }
        
        return;
    }
    
    // Update direction and compute desired speed
    m->speed.direction = new_direction;
    float f_distance = (float)abs(distance);
    m->speed.deceleration_distance = (m->speed.current_speed * m->speed.current_speed) / 
                                     (2.0f * m->speed.deceleration);
    
    float desired_speed;
    if (f_distance < m->speed.deceleration_distance) {
        desired_speed = sqrtf(2.0f * m->speed.deceleration * f_distance);
    } else {
        desired_speed = m->speed.max_speed;
    }
    
    desired_speed *= m->speed.direction;
    
    // Accelerate or decelerate toward desired speed
    if (m->speed.current_speed < desired_speed) {
        m->speed.current_speed += m->speed.acceleration * m->timing.dt;
        if (m->speed.current_speed > desired_speed) {
            m->speed.current_speed = desired_speed;
        }
    } else if (m->speed.current_speed > desired_speed) {
        m->speed.current_speed -= m->speed.deceleration * m->timing.dt;
        if (m->speed.current_speed < desired_speed) {
            m->speed.current_speed = desired_speed;
        }
    }
}

// Execute a step based on current speed
static void motor_execute_step(StepperControl* m) {
    if (m->speed.current_speed == 0) return;
    
    float step_interval_s = 1.0f / fabsf(m->speed.current_speed);
    int64_t now_us = esp_timer_get_time();
    
    // Check if it's time to step
    if ((now_us - m->timing.last_update_us) >= step_interval_s * 1000000.0f) {
        portENTER_CRITICAL(&motor_spinlock);
        m->current_step += m->speed.direction;
        
        // Wrap around for angle wrap (corrected from shortest_path)
        if (m->enable_angle_wrap) {
            if (m->current_step >= MOTOR_STEPS_PER_REV) {
                m->current_step -= MOTOR_STEPS_PER_REV;
            } else if (m->current_step < 0) {
                m->current_step += MOTOR_STEPS_PER_REV;
            }
        }
        
        // Set GPIOs according to step sequence
        int seq_index = (m->current_step % MOTOR_STEP_SEQUENCE_COUNT + 
                        MOTOR_STEP_SEQUENCE_COUNT) % MOTOR_STEP_SEQUENCE_COUNT;
        motor_set_pins(m, seq_index);
        
        // Update current angle and timestamp
        m->current_angle = motor_steps_to_deg(m->current_step);
        m->timing.last_update_us = now_us;
        portEXIT_CRITICAL(&motor_spinlock);
    }
}

// Set deceleration value for the motor
void stepper_set_deceleration(StepperControl* m, float deceleration) {
    if (deceleration > 0) {
        m->speed.deceleration = deceleration;
        ESP_LOGI("MOTOR", "Deceleration set to: %.1f steps/s²", deceleration);
    }
}

// Enable or disable angle wrap
void stepper_enable_angle_wrap(StepperControl* m, bool enable) {
    m->enable_angle_wrap = enable;
    ESP_LOGI("MOTOR", "Angle wrap %s", enable ? "enabled" : "disabled");
}

// Log motor status periodically
static void motor_log_status(StepperControl* m) {
    int64_t now_us = esp_timer_get_time();
    
    // Only log at defined intervals
    if ((now_us - m->timing.last_log_us) >= (MOTOR_LOG_INTERVAL_MS * 1000)) {
        if (m->is_moving || m->new_target_received) {
            ESP_LOGI("MOTOR", "Angle: %.1f°/%.1f°, Speed: %.1f steps/s, Moving: %s", 
                    m->current_angle, m->target_angle, 
                    m->speed.current_speed,
                    m->is_moving ? "YES" : "NO");
        }
        m->timing.last_log_us = now_us;
    }
}

// Initialize stepper motor and GPIO pins
void stepper_init(StepperControl* m, int p1, int p2, int p3, int p4,
                  float maxSpeedDegPerSec, float maxAccelDegPerSec2,
                  motor_speed_profile_t speed_profile, float default_angle) {
    m->pin1 = p1; m->pin2 = p2; m->pin3 = p3; m->pin4 = p4;
    
    // Reset steps and angles
    m->current_step = 0;
    m->target_step = 0;
    m->shortest_path_step = 0;
    m->current_angle = 0.0f;
    m->target_angle = 0.0f;
    m->default_angle = default_angle;
    
    // Control flags
    m->enable_shortest_path = MOTOR_ENABLE_SHORTEST_PATH;
    m->enable_angle_wrap = MOTOR_ENABLE_ANGLE_WRAP;
    m->is_moving = false;
    m->new_target_received = false;
    
    // Initialize timing
    m->timing.last_update_us = esp_timer_get_time();
    m->timing.last_log_us = m->timing.last_update_us;
    m->timing.dt = 0.0f;
    m->timing.task_run_count = 0;
    
    // Initialize speed planner
    m->speed.profile = speed_profile;
    m->speed.current_speed = 0.0f;
    m->speed.target_speed = 0.0f;
    m->speed.max_speed = (maxSpeedDegPerSec / MOTOR_MAX_ANGLE_DEG) * MOTOR_STEPS_PER_REV;
    m->speed.acceleration = (maxAccelDegPerSec2 / MOTOR_MAX_ANGLE_DEG) * MOTOR_STEPS_PER_REV;
    m->speed.deceleration = m->speed.acceleration;
    m->speed.deceleration_distance = 0.0f;
    m->speed.direction = 1;
    m->speed.is_decelerating = false;
    
    // Configure GPIO pins
    int pins[4] = {p1, p2, p3, p4};
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .pin_bit_mask = 0
    };
    
    for(int i = 0; i < 4; i++) {
        io_conf.pin_bit_mask |= (1ULL << pins[i]);
    }
    gpio_config(&io_conf);
    
    // Set initial position to default angle
    stepper_set_current_angle(m, default_angle);
    
    ESP_LOGI("MOTOR", "Stepper motor initialized successfully");
    ESP_LOGI("MOTOR", "Steps per revolution: %d", MOTOR_STEPS_PER_REV);
    ESP_LOGI("MOTOR", "Step sequence count: %d", MOTOR_STEP_SEQUENCE_COUNT);
    ESP_LOGI("MOTOR", "Max speed: %.1f steps/s", m->speed.max_speed);
    ESP_LOGI("MOTOR", "Acceleration: %.1f steps/s²", m->speed.acceleration);
    ESP_LOGI("MOTOR", "Shortest path enabled: %s", m->enable_shortest_path ? "YES" : "NO");
    ESP_LOGI("MOTOR", "Angle wrap enabled: %s", m->enable_angle_wrap ? "YES" : "NO");
    ESP_LOGI("MOTOR", "Default angle: %.1f°", default_angle);
}

// Move motor to specified angle
void stepper_move_to_angle(StepperControl* m, float angle) {
    // Clamp angle within valid range
    if (angle < 0) angle = 0;
    if (angle > MOTOR_MAX_ANGLE_DEG) angle = MOTOR_MAX_ANGLE_DEG;
    
    portENTER_CRITICAL(&motor_spinlock);
    m->target_angle = angle;
    m->new_target_received = true;
    motor_calculate_shortest_path(m, angle);
    portEXIT_CRITICAL(&motor_spinlock);
    
    ESP_LOGI("MOTOR", "Move to angle: %.1f°", angle);
}

// Set default angle for motor
void stepper_set_default_angle(StepperControl* m, float angle) {
    if (angle < 0) angle = 0;
    if (angle > MOTOR_MAX_ANGLE_DEG) angle = MOTOR_MAX_ANGLE_DEG;
    m->default_angle = angle;
}

// Move motor back to default angle
void stepper_return_to_default(StepperControl* m) {
    stepper_move_to_angle(m, m->default_angle);
}

// Get current motor angle
float stepper_get_current_angle(StepperControl* m) {
    return m->current_angle;
}

// Set current angle directly, reset speed and steps
void stepper_set_current_angle(StepperControl* m, float angle) {
    if (angle < 0) angle = 0;
    if (angle > MOTOR_MAX_ANGLE_DEG) angle = MOTOR_MAX_ANGLE_DEG;
    
    portENTER_CRITICAL(&motor_spinlock);
    m->current_angle = angle;
    m->target_angle = angle;
    m->current_step = motor_deg_to_steps(angle);
    m->target_step = m->current_step;
    m->speed.current_speed = 0.0f;
    m->is_moving = false;
    portEXIT_CRITICAL(&motor_spinlock);
}

// Stop motor immediately
void stepper_stop(StepperControl* m) {
    portENTER_CRITICAL(&motor_spinlock);
    m->target_step = m->current_step;
    m->target_angle = m->current_angle;
    m->speed.current_speed = 0.0f;
    m->is_moving = false;
    m->new_target_received = false;
    portEXIT_CRITICAL(&motor_spinlock);
    
    ESP_LOGI("MOTOR", "Motor stopped at %.1f°", m->current_angle);
}

// Check if motor is currently moving
bool stepper_is_moving(StepperControl* m) {
    return m->is_moving;
}

// Enable or disable shortest path rotation
void stepper_enable_shortest_path(StepperControl* m, bool enable) {
    m->enable_shortest_path = enable;
    ESP_LOGI("MOTOR", "Shortest path %s", enable ? "enabled" : "disabled");
}

// Change speed profile (trapezoid, S-curve, constant)
void stepper_set_speed_profile(StepperControl* m, motor_speed_profile_t profile) {
    m->speed.profile = profile;
    ESP_LOGI("MOTOR", "Speed profile changed to: %d", profile);
}

// Main motor task: update timing, plan speed, execute steps, log status
void stepper_run(StepperControl* m) {
    motor_update_timing(m);
    
    switch (m->speed.profile) {
        case MOTOR_SPEED_TRAPEZOIDAL:
            motor_trapezoidal_speed_planning(m);
            break;
        case MOTOR_SPEED_S_CURVE:
            motor_trapezoidal_speed_planning(m); // TODO: Implement S-curve
            break;
        case MOTOR_SPEED_CONSTANT:
            motor_trapezoidal_speed_planning(m); // TODO: Implement constant speed
            break;
    }
    
    motor_execute_step(m);
    motor_log_status(m);
}

// Print motor performance statistics
void stepper_print_performance(StepperControl* m) {
    ESP_LOGI("MOTOR", "=== Motor Performance ===");
    ESP_LOGI("MOTOR", "Task run count: %lu", m->timing.task_run_count);
    ESP_LOGI("MOTOR", "Current angle: %.1f°", m->current_angle);
    ESP_LOGI("MOTOR", "Target angle: %.1f°", m->target_angle);
    ESP_LOGI("MOTOR", "Current speed: %.1f steps/s", m->speed.current_speed);
    ESP_LOGI("MOTOR", "Is moving: %s", m->is_moving ? "YES" : "NO");
    ESP_LOGI("MOTOR", "Shortest path: %s", m->enable_shortest_path ? "YES" : "NO");
    ESP_LOGI("MOTOR", "Angle wrap: %s", m->enable_angle_wrap ? "YES" : "NO");
}