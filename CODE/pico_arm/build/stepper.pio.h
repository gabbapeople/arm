// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------- //
// stepper //
// ------- //

#define stepper_wrap_target 0
#define stepper_wrap 5

static const uint16_t stepper_program_instructions[] = {
            //     .wrap_target
    0x0020, //  0: jmp    !x, 0                      
    0xb846, //  1: mov    y, isr          side 1     
    0x0082, //  2: jmp    y--, 2                     
    0xb046, //  3: mov    y, isr          side 0     
    0x0084, //  4: jmp    y--, 4                     
    0x0040, //  5: jmp    x--, 0                     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program stepper_program = {
    .instructions = stepper_program_instructions,
    .length = 6,
    .origin = -1,
};

static inline pio_sm_config stepper_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + stepper_wrap_target, offset + stepper_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}

static inline void stepper_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = stepper_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    pio_sm_init(pio, sm, offset, &c);
}

#endif
