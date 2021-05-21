
#include "stepper.h"

StepperDriver stepper_driver;
Stepper stepper_0;
Stepper stepper_1;

void stepper_driver_init() {
    stepper_driver.pio = pio0;
    uint offset = pio_add_program(stepper_driver.pio, &stepper_program);
    stepper_driver.offset = offset;
}

void stepper_0_init() {
    stepper_0.step_pin = STEPPER_0_STEP_GPIO_PIN;
    stepper_0.dir_pin = STEPPER_0_DIR_GPIO_PIN;

    gpio_init(stepper_0.dir_pin);
    gpio_set_dir(stepper_0.dir_pin, GPIO_OUT);

    uint sm = 0;
    stepper_0.sm = sm;
    stepper_program_init(stepper_driver.pio, stepper_0.sm, stepper_driver.offset, stepper_0.step_pin);
}

void stepper_1_init() {
    stepper_1.step_pin = STEPPER_1_STEP_GPIO_PIN;
    stepper_1.dir_pin = STEPPER_1_DIR_GPIO_PIN;

    gpio_init(stepper_1.dir_pin);
    gpio_set_dir(stepper_1.dir_pin, GPIO_OUT);

    uint sm = 1;
    stepper_1.sm = sm;
    stepper_program_init(stepper_driver.pio, stepper_1.sm, stepper_driver.offset, stepper_1.step_pin);
}

void stepper_move(uint device_id, uint32_t frequency, uint32_t steps) { 

    uint32_t us = 1000000 / frequency;
    us = us / 2;
    if (us < 5) us = 5;

    uint32_t cycles = us * (clock_get_hz(clk_sys) / 1000000);
    uint32_t phases = steps;

    uint sm = device_id;

    pio_sm_set_enabled(stepper_driver.pio, sm, false);

    pio_sm_put_blocking(stepper_driver.pio, sm, cycles);
    pio_sm_exec(stepper_driver.pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(stepper_driver.pio, sm, pio_encode_out(pio_isr, 32));
    
    pio_sm_put_blocking(stepper_driver.pio, sm, phases);
    pio_sm_exec(stepper_driver.pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(stepper_driver.pio, sm, pio_encode_out(pio_x, 32));

    pio_sm_set_enabled(stepper_driver.pio, sm, true);
}

void move(uint device_id, int16_t steps) { 

    uint32_t speed = 0;
    uint32_t abs_steps = steps < 0 ? -1 * steps : steps;

    uint32_t stepper_pps_min = 0;
    uint32_t stepper_pps_max = 0;

    switch (device_id) {
    case 0:
#if CONTROLLER_ID == 0
        stepper_pps_min = STEPPER_0_PPS_MIN;
        stepper_pps_max = STEPPER_0_PPS_MAX;
#endif     
#if CONTROLLER_ID == 1
        stepper_pps_min = STEPPER_2_PPS_MIN;
        stepper_pps_max = STEPPER_2_PPS_MAX;
#endif
#if CONTROLLER_ID == 2
        stepper_pps_min = STEPPER_4_PPS_MIN;
        stepper_pps_max = STEPPER_4_PPS_MAX;
#endif
        break;
    case 1:
#if CONTROLLER_ID == 0
        stepper_pps_min = STEPPER_1_PPS_MIN;
        stepper_pps_max = STEPPER_1_PPS_MAX;
#endif
#if CONTROLLER_ID == 1
        stepper_pps_min = STEPPER_3_PPS_MIN;
        stepper_pps_max = STEPPER_3_PPS_MAX;
#endif
        break;
    default:
        break;
    }

    speed = stepper_pps_min;

    bool dir = steps > 0 ? true : false;
 
    uint dir_pin = 0;
    switch (device_id) {
    case 0:
        dir_pin = stepper_0.dir_pin;
        break;
    case 1:
        dir_pin = stepper_1.dir_pin;
        break;
    default:
        break;
    }

    gpio_put(dir_pin, dir);
    stepper_move(device_id, speed, abs_steps);

}

void stop(uint device_id) {
    uint sm = device_id;
    pio_sm_set_enabled(stepper_driver.pio, sm, false);
}