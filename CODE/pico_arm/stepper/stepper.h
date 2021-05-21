
#ifndef STEPPER_H
#define STEPPER_H

#include <stdio.h>

#include "pico/stdlib.h"

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "stepper.pio.h"

#define CONTROLLER_ID 2

#define STEPPER_0_STEP_GPIO_PIN 10
#define STEPPER_0_DIR_GPIO_PIN 11

#define STEPPER_1_STEP_GPIO_PIN 12
#define STEPPER_1_DIR_GPIO_PIN 13

#define STEPPER_0_PPS_MIN 300
#define STEPPER_0_PPS_MAX 600

#define STEPPER_1_PPS_MIN 300
#define STEPPER_1_PPS_MAX 600

#define STEPPER_2_PPS_MIN 300
#define STEPPER_2_PPS_MAX 600

#define STEPPER_3_PPS_MIN 300
#define STEPPER_3_PPS_MAX 600

#define STEPPER_4_PPS_MIN 300
#define STEPPER_4_PPS_MAX 600

typedef struct {
    PIO pio;
    uint offset;
} StepperDriver;

typedef struct {
    uint step_pin;
    uint dir_pin;
    uint sm;
} Stepper;

void stepper_driver_init();
void stepper_0_init();
void stepper_1_init();

void stepper_move(uint device_id, uint32_t frequency, uint32_t steps);

void move(uint device_id, int16_t steps);
void stop(uint device_id);

#endif // STEPPER
