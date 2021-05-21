
#include "servo.h"

ServoDriver servo_driver;
Servo servo_0;
Servo servo_1;

void servo_driver_init() {
    servo_driver.pio = pio1;
    uint offset = pio_add_program(servo_driver.pio, &servo_program);
    servo_driver.offset = offset;
}

void servo_0_init(uint8_t pin) {
    servo_0.sm = 0;
    servo_0.pin = pin;
    servo_program_init(servo_driver.pio, servo_0.sm, servo_driver.offset, servo_0.pin);
}

void servo_1_init(uint8_t pin) {
    servo_1.sm = 1;
    servo_1.pin = pin;
    servo_program_init(servo_driver.pio, servo_1.sm, servo_driver.offset, servo_1.pin);
}

void servo_attach(uint8_t device_id) {
    uint sm = device_id;

    pio_sm_set_enabled(servo_driver.pio, sm, false);

    uint32_t hz = 50;
    uint32_t us = 1000000 / hz;
    uint32_t period = us * (clock_get_hz(clk_sys) / 1000000) / 3;

    pio_sm_put_blocking(servo_driver.pio, sm, period);
    pio_sm_exec(servo_driver.pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(servo_driver.pio, sm, pio_encode_out(pio_isr, 32));
    pio_sm_set_enabled(servo_driver.pio, sm, true);
}

void servo_write_microseconds(uint8_t device_id, uint32_t value) {
    uint sm = device_id;
    pio_sm_put_blocking(servo_driver.pio, sm, value * (clock_get_hz(clk_sys) / 1000000) / 3);
}