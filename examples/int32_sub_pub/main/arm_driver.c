#include "arm_driver.h"
#include "params.h"
#include "pwm_driver.h"  // 你要實作這個來對接 PCA9685
#include <stdlib.h>

static uint16_t map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (uint16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void set_servo_angle(uint8_t servo_num, float angle) {
    if (servo_num < NUM_OF_SERVOS) {
        uint16_t pulse = map_float(angle, 0, 180, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
        pwm_write_microseconds(servo_num, pulse);  // 實作此函式與 PCA9685 驅動連接
    }
}

void arm_manager_init(ArmManager *arm, uint8_t num_servos, const uint8_t *min, const uint8_t *max, const uint8_t *init) {
    arm->num_servos = num_servos;
    arm->servo_target_angles = malloc(num_servos);
    arm->servo_current_angles = malloc(sizeof(float) * num_servos);
    arm->servo_min_angles = malloc(num_servos);
    arm->servo_max_angles = malloc(num_servos);

    pwm_begin();  // 初始化 PCA9685
    pwm_set_oscillator_frequency(27000000);
    pwm_set_pwm_freq(50);

    for (uint8_t i = 0; i < num_servos; ++i) {
        arm->servo_min_angles[i] = min[i];
        arm->servo_max_angles[i] = max[i];
        arm->servo_target_angles[i] = init[i];
        arm->servo_current_angles[i] = (float)init[i] + 1;
    }
}

void arm_manager_set_servo_target_angle(ArmManager *arm, uint8_t servo_num, uint8_t target_angle) {
    if (servo_num < arm->num_servos) {
        uint8_t constrained = target_angle;
        if (constrained < arm->servo_min_angles[servo_num]) constrained = arm->servo_min_angles[servo_num];
        if (constrained > arm->servo_max_angles[servo_num]) constrained = arm->servo_max_angles[servo_num];
        arm->servo_target_angles[servo_num] = constrained;
    }
}

void arm_manager_change_servo_target_angle(ArmManager *arm, uint8_t servo_num, int8_t bias_angle) {
    if (servo_num < arm->num_servos) {
        int16_t new_angle = arm->servo_target_angles[servo_num] + bias_angle;
        if (new_angle < arm->servo_min_angles[servo_num]) new_angle = arm->servo_min_angles[servo_num];
        if (new_angle > arm->servo_max_angles[servo_num]) new_angle = arm->servo_max_angles[servo_num];
        arm->servo_target_angles[servo_num] = (uint8_t)new_angle;
    }
}

void arm_manager_get_current_angles(ArmManager *arm, float *current_angles) {
    for (uint8_t i = 0; i < arm->num_servos; ++i) {
        current_angles[i] = arm->servo_current_angles[i];
    }
}

void arm_manager_move_arm(ArmManager *arm) {
    for (uint8_t i = 0; i < arm->num_servos; ++i) {
        float diff = arm->servo_target_angles[i] - arm->servo_current_angles[i];
        if (diff >= ARM_MOVEMENT_STEP || diff <= -ARM_MOVEMENT_STEP) {
            float step = (diff > 0) ? ARM_MOVEMENT_STEP : -ARM_MOVEMENT_STEP;
            arm->servo_current_angles[i] += step;
            set_servo_angle(i, arm->servo_current_angles[i]);
        }
    }
}
