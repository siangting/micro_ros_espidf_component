
#include "pwm_driver.h"
#include <stdio.h>

void pwm_begin(void) {
    printf("PWM begin\n");
}

void pwm_set_oscillator_frequency(uint32_t freq) {
    printf("Set oscillator freq: %lu\n", freq);
}

void pwm_set_pwm_freq(uint16_t freq) {
    printf("Set PWM freq: %u\n", freq);
}

void pwm_write_microseconds(uint8_t channel, uint16_t microseconds) {
    printf("PWM channel %d: %d us\n", channel, microseconds);
}
