#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "servo.pio.h"

typedef struct {
    PIO pio;
    uint offset;
} ServoDriver;

typedef struct {
    uint pin;
    uint sm;
} Servo;

void servo_driver_init();
void servo_0_init(uint8_t pin);
void servo_1_init(uint8_t pin);
void servo_attach(uint8_t device_id);
void servo_write_microseconds(uint8_t device_id, uint32_t value);

#endif // SERVO_H