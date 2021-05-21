
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

#include "stepper.h"
#include "as5048a/as5048a.h"
#include "servo/servo.h"

#define CONTROLLER_ID 2
#define DEBUG 1
#define MAIN_LOOP_RATE_HZ 100

#if CONTROLLER_ID == 2
#include "servo/servo.h"
#endif

// UART & PROTOCOL TURNING
#define UART_ID uart1
#define UART_BAUD_RATE 115200
#define UART_TX_PIN 4
#define UART_RX_PIN 5 

#define HEAD_BYTE 0x20
#define CHECKSUM_BYTE 0x40
#define PACKET_SIZE 10

//// VARIABLES ////
// CONTROLLER COMMON VARS & TYPES
const int16_t ENCODER_TOTAL_PULSES = AS5048A_RESOLUTION -  1;
const double ENCODER_DEGREE_IN_PULSE = 360.0 / ENCODER_TOTAL_PULSES;
const double ENCODER_HALF_TOTAL_PULSES = 8191.0;

typedef union TargetPosition{
    float f;
    uint8_t b[4]; 
} TargetPosition;

TargetPosition tp_0 = { 0 };
TargetPosition tp_1 = { 0 };

uint8_t read_counter = 0;
bool is_header = false;
bool first_time_header = false;
uint8_t packet[PACKET_SIZE];

#if CONTROLLER_ID == 0 // CONTROLLER 0
// CONTROLLER ENCODER
uint16_t ENCODER_0_CURRENT_PULSES = 0;
uint16_t ENCODER_1_CURRENT_PULSES = 0;
int16_t ENCODER_0_CURRENT_NORM_PULSES = 0;
int16_t ENCODER_1_CURRENT_NORM_PULSES = 0;
int16_t J0_CURRENT_STEPS = 0;
int16_t J1_CURRENT_STEPS = 0;
double J0_CURRENT_ANGLE_DEGREE = 0.0;
double J1_CURRENT_ANGLE_DEGREE = 0.0;
uint16_t ENCODER_0_ZERO_POSITION = 3663 % 0x3FFF;
uint16_t ENCODER_1_ZERO_POSITION = 11116 % 0x3FFF;

// CONTROLLER INPUT J0
const double J0_MICROSTEP_MULTIPLY = 1.0;
const double J0_STEPS = 2300;
const double J0_STEPPER_TOTAL_STEPS = J0_STEPS * J0_MICROSTEP_MULTIPLY; // 2300
const double J0_DEGREE_IN_STEP = 360.0 / J0_STEPPER_TOTAL_STEPS; // 0.156521739
const double J0_TARGET_MIN_STEPS = -1140.0;
const double J0_TARGET_MAX_STEPS = 1140.0;
const double J0_TARGET_MIN_ANGLE_DEGREE = J0_TARGET_MIN_STEPS * J0_DEGREE_IN_STEP; // -178.43478246
const double J0_TARGET_MAX_ANGLE_DEGREE = J0_TARGET_MAX_STEPS * J0_DEGREE_IN_STEP; // 178.43478246
double J0_TARGET_ANGLE_DEGREE = 0.0;
int16_t J0_TARGET_STEPS = 0;
int16_t J0_STEPS_TO_GO = 0;
bool J0_REACHED = false;
int16_t J0_PREVIOUS_TARGET_STEPS = 0;

// CONTROLLER INPUT J1
const double J1_MICROSTEP_MULTIPLY = 1.0;
const double J1_STEPS = 3000;
const double J1_STEPPER_TOTAL_STEPS = J1_STEPS * J1_MICROSTEP_MULTIPLY; // 3000
const double J1_DEGREE_IN_STEP = 360.0 / J0_STEPPER_TOTAL_STEPS; // 0.12
const double J1_TARGET_MIN_STEPS = -959.0;
const double J1_TARGET_MAX_STEPS = 959.0;
const double J1_TARGET_MIN_ANGLE_DEGREE = J1_TARGET_MIN_STEPS * J1_DEGREE_IN_STEP; // -115.08
const double J1_TARGET_MAX_ANGLE_DEGREE = J1_TARGET_MAX_STEPS * J1_DEGREE_IN_STEP; // 115.08
double J1_TARGET_ANGLE_DEGREE = 0.0;
int16_t J1_TARGET_STEPS = 0;
int16_t J1_STEPS_TO_GO = 0;
bool J1_REACHED = false;
int16_t J1_PREVIOUS_TARGET_STEPS = 0;
#endif 

#if CONTROLLER_ID == 1 // CONTROLLER 1
// CONTROLLER ENCODER
uint16_t ENCODER_2_CURRENT_PULSES = 0;
uint16_t ENCODER_3_CURRENT_PULSES = 0;
int16_t ENCODER_2_CURRENT_NORM_PULSES = 0;
int16_t ENCODER_3_CURRENT_NORM_PULSES = 0;
int16_t J2_CURRENT_STEPS = 0;
int16_t J3_CURRENT_STEPS = 0;
double J2_CURRENT_ANGLE_DEGREE = 0.0;
double J3_CURRENT_ANGLE_DEGREE = 0.0;
uint16_t ENCODER_2_ZERO_POSITION = 14040 % 0x3FFF;
uint16_t ENCODER_3_ZERO_POSITION = 1637 % 0x3FFF;

// CONTROLLER INPUT J2
const double J2_MICROSTEP_MULTIPLY = 1.0;
const double J2_STEPS = 2000;
const double J2_STEPPER_TOTAL_STEPS = J2_STEPS * J2_MICROSTEP_MULTIPLY; // 2000
const double J2_DEGREE_IN_STEP = 360.0 / J2_STEPPER_TOTAL_STEPS; // 0.18
const double J2_TARGET_MIN_STEPS = -723.0;
const double J2_TARGET_MAX_STEPS = 723.0;
const double J2_TARGET_MIN_ANGLE_DEGREE = J2_TARGET_MIN_STEPS * J2_DEGREE_IN_STEP; // -130.14
const double J2_TARGET_MAX_ANGLE_DEGREE = J2_TARGET_MAX_STEPS * J2_DEGREE_IN_STEP; // 130.14
double J2_TARGET_ANGLE_DEGREE = 0.0;
int16_t J2_TARGET_STEPS = 0;
int16_t J2_STEPS_TO_GO = 0;
bool J2_REACHED = false;
int16_t J2_PREVIOUS_TARGET_STEPS = 0;

// CONTROLLER INPUT J3
const double J3_MICROSTEP_MULTIPLY = 1.0;
const double J3_STEPS = 2812.5;
const double J3_STEPPER_TOTAL_STEPS = J3_STEPS * J3_MICROSTEP_MULTIPLY; // 2812.5
const double J3_DEGREE_IN_STEP = 360.0 / J3_STEPPER_TOTAL_STEPS; // 0.128
const double J3_TARGET_MIN_STEPS = -1396.0;
const double J3_TARGET_MAX_STEPS = 1396.0;
const double J3_TARGET_MIN_ANGLE_DEGREE = J3_TARGET_MIN_STEPS * J3_DEGREE_IN_STEP; // -178.688
const double J3_TARGET_MAX_ANGLE_DEGREE = J3_TARGET_MAX_STEPS * J3_DEGREE_IN_STEP; // 178.688
double J3_TARGET_ANGLE_DEGREE = 0.0;
double J3_PREV_TARGET_ANGLE_DEGREE = 0.0;
int16_t J3_TARGET_STEPS = 0;
int16_t J3_STEPS_TO_GO = 0;
bool J3_REACHED = false;
int16_t J3_PREVIOUS_TARGET_STEPS = 0;
#endif 

#if CONTROLLER_ID == 2 // CONTROLLER 2
// SERVO JOINT
const uint8_t J5_SERVO_GPIO_PIN = 6;
uint32_t J5_TARGET_US = 1500;

double J5_TARGET_ANGLE_DEGREE = 0.0;
double J5_TARGET_MIN_ANGLE_DEGREE = -90.0;
double J5_TARGET_MAX_ANGLE_DEGREE = 90.0;

uint32_t J5_ZERO_US = 1500; // ~ 0
uint32_t J5_MIN_US = 1156; // ~ -90
uint32_t J5_MAX_US = 1857; // ~ 90

// GRIPPER
const uint8_t GRIPPER_SERVO_GPIO_PIN = 7;
uint16_t GRIPPER_TARGET_US = 1500;

// CONTROLLER ENCODER
uint16_t ENCODER_4_CURRENT_PULSES = 0;
int16_t ENCODER_4_CURRENT_NORM_PULSES = 0;
int16_t J4_CURRENT_STEPS = 0;
double J4_CURRENT_ANGLE_DEGREE = 0.0;
uint16_t ENCODER_4_ZERO_POSITION = 7694 % 0x3FFF;

// CONTROLLER INPUT J4
const double J4_MICROSTEP_MULTIPLY = 1.0;
const double J4_STEPS = 1406.25;
const double J4_STEPPER_TOTAL_STEPS = J4_STEPS * J4_MICROSTEP_MULTIPLY; // 1406.25
const double J4_DEGREE_IN_STEP = 360.0 / J4_STEPPER_TOTAL_STEPS; // 0.256
const double J4_TARGET_MIN_STEPS = -508;
const double J4_TARGET_MAX_STEPS = 508;
const double J4_TARGET_MIN_ANGLE_DEGREE = J4_TARGET_MIN_STEPS * J4_DEGREE_IN_STEP; // -130,048
const double J4_TARGET_MAX_ANGLE_DEGREE = J4_TARGET_MAX_STEPS * J4_DEGREE_IN_STEP; // 130,048
double J4_TARGET_ANGLE_DEGREE = 0.0;
int16_t J4_TARGET_STEPS = 0;
int16_t J4_STEPS_TO_GO = 0;
bool J4_REACHED = false;
int16_t J4_PREVIOUS_TARGET_STEPS = 0;
#endif 

//// FUNCTIONS ////
// ENCODER
int16_t invert_pulse(uint16_t pulse) {
    return ENCODER_TOTAL_PULSES - pulse;
}

uint16_t get_current_pulses(uint8_t device_id) {
    uint16_t pulses = as5048a_get_simple_avr_raw_rotation(device_id, 128);
    return pulses;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#if CONTROLLER_ID == 0 // CONTROLLER 0
void calculate_target_position() {
    J0_TARGET_ANGLE_DEGREE = tp_0.f;
    J0_TARGET_STEPS = round(J0_TARGET_ANGLE_DEGREE / J0_DEGREE_IN_STEP);

    if (J0_TARGET_STEPS < J0_TARGET_MIN_STEPS)
        J0_TARGET_STEPS =  J0_TARGET_MIN_STEPS;

    if (J0_TARGET_STEPS > J0_TARGET_MAX_STEPS)
        J0_TARGET_STEPS =  J0_TARGET_MAX_STEPS;

    J1_TARGET_ANGLE_DEGREE = tp_1.f;
    J1_TARGET_STEPS = round(J1_TARGET_ANGLE_DEGREE / J1_DEGREE_IN_STEP);

    if (J1_TARGET_STEPS < J1_TARGET_MIN_STEPS)
        J1_TARGET_STEPS =  J1_TARGET_MIN_STEPS;

    if (J1_TARGET_STEPS > J1_TARGET_MAX_STEPS)
        J1_TARGET_STEPS =  J1_TARGET_MAX_STEPS;

}

void calculate_current_position() {
    ENCODER_0_CURRENT_PULSES = get_current_pulses(0);
    ENCODER_1_CURRENT_PULSES = get_current_pulses(1);

    int16_t rotation = 0;
    
    rotation = (int16_t)ENCODER_0_CURRENT_PULSES - (int16_t)ENCODER_0_ZERO_POSITION;
	if (rotation > ENCODER_HALF_TOTAL_PULSES)
		rotation = -((0x3FFF) - rotation);

    ENCODER_0_CURRENT_NORM_PULSES = rotation;

    J0_CURRENT_ANGLE_DEGREE = ENCODER_DEGREE_IN_PULSE * ENCODER_0_CURRENT_NORM_PULSES;
    
    J0_CURRENT_ANGLE_DEGREE  = -1 * J0_CURRENT_ANGLE_DEGREE; //INVERT!

    J0_CURRENT_STEPS = round(J0_CURRENT_ANGLE_DEGREE / J0_DEGREE_IN_STEP);

    rotation = (int16_t)ENCODER_1_CURRENT_PULSES - (int16_t)ENCODER_1_ZERO_POSITION;
	if (rotation > ENCODER_HALF_TOTAL_PULSES)
		rotation = -((0x3FFF) - rotation);

    ENCODER_1_CURRENT_NORM_PULSES = rotation;

    J1_CURRENT_ANGLE_DEGREE = ENCODER_DEGREE_IN_PULSE * ENCODER_1_CURRENT_NORM_PULSES;
    J1_CURRENT_STEPS = round(J1_CURRENT_ANGLE_DEGREE / J1_DEGREE_IN_STEP);
}
#endif

#if CONTROLLER_ID == 1 // CONTROLLER 1
void calculate_target_position() {
    J2_TARGET_ANGLE_DEGREE = tp_0.f;
    J2_TARGET_STEPS = round(J2_TARGET_ANGLE_DEGREE / J2_DEGREE_IN_STEP);

    if (J2_TARGET_STEPS < J2_TARGET_MIN_STEPS)
        J2_TARGET_STEPS =  J2_TARGET_MIN_STEPS;

    if (J2_TARGET_STEPS > J2_TARGET_MAX_STEPS)
        J2_TARGET_STEPS =  J2_TARGET_MAX_STEPS;

    J3_TARGET_ANGLE_DEGREE = tp_1.f;
    J3_TARGET_STEPS = round(J3_TARGET_ANGLE_DEGREE / J3_DEGREE_IN_STEP);

    if (J3_TARGET_STEPS < J3_TARGET_MIN_STEPS)
        J3_TARGET_STEPS =  J3_TARGET_MIN_STEPS;

    if (J3_TARGET_STEPS > J3_TARGET_MAX_STEPS)
        J3_TARGET_STEPS =  J3_TARGET_MAX_STEPS;
}

void calculate_current_position() {
    ENCODER_2_CURRENT_PULSES = get_current_pulses(0);
    ENCODER_3_CURRENT_PULSES = get_current_pulses(1);

    int16_t rotation = 0;

    rotation = (int16_t)ENCODER_2_CURRENT_PULSES - (int16_t)ENCODER_2_ZERO_POSITION;
	if (rotation > ENCODER_HALF_TOTAL_PULSES)
		rotation = -((0x3FFF) - rotation);
    if (rotation < -0x1FFF)
        rotation = rotation + 0x3FFF;

    ENCODER_2_CURRENT_NORM_PULSES = rotation;

    J2_CURRENT_ANGLE_DEGREE = ENCODER_DEGREE_IN_PULSE * ENCODER_2_CURRENT_NORM_PULSES;
    
    J2_CURRENT_ANGLE_DEGREE  = -1 * J2_CURRENT_ANGLE_DEGREE; //INVERT!

    J2_CURRENT_STEPS = round(J2_CURRENT_ANGLE_DEGREE / J2_DEGREE_IN_STEP);

    rotation = (int16_t)ENCODER_3_CURRENT_PULSES - (int16_t)ENCODER_3_ZERO_POSITION;
	if (rotation > ENCODER_HALF_TOTAL_PULSES)
		rotation = -((0x3FFF) - rotation);
    if (rotation < -0x1FFF)
        rotation = rotation + 0x3FFF;

    ENCODER_3_CURRENT_NORM_PULSES = rotation;

    J3_CURRENT_ANGLE_DEGREE = ENCODER_DEGREE_IN_PULSE * ENCODER_3_CURRENT_NORM_PULSES;

    J3_CURRENT_ANGLE_DEGREE  = -1 * J3_CURRENT_ANGLE_DEGREE;

    J3_CURRENT_STEPS = round(J3_CURRENT_ANGLE_DEGREE / J3_DEGREE_IN_STEP);
}
#endif

#if CONTROLLER_ID == 2 // CONTROLLER 1
void calculate_target_position() {
    J4_TARGET_ANGLE_DEGREE = tp_0.f;
    J4_TARGET_STEPS = round(J4_TARGET_ANGLE_DEGREE / J4_DEGREE_IN_STEP);

    if (J4_TARGET_STEPS < J4_TARGET_MIN_STEPS)
        J4_TARGET_STEPS =  J4_TARGET_MIN_STEPS;

    if (J4_TARGET_STEPS > J4_TARGET_MAX_STEPS)
        J4_TARGET_STEPS =  J4_TARGET_MAX_STEPS;

    J5_TARGET_ANGLE_DEGREE = tp_1.f;

    if (J5_TARGET_ANGLE_DEGREE < J5_TARGET_MIN_ANGLE_DEGREE)
        J5_TARGET_ANGLE_DEGREE = J5_TARGET_MIN_ANGLE_DEGREE;
    if (J5_TARGET_ANGLE_DEGREE > J5_TARGET_MAX_ANGLE_DEGREE)
        J5_TARGET_ANGLE_DEGREE = J5_TARGET_MAX_ANGLE_DEGREE;

    if (J5_TARGET_ANGLE_DEGREE >= 0) {
        J5_TARGET_US = (J5_TARGET_ANGLE_DEGREE - 0) * (J5_MAX_US - J5_ZERO_US) / (J5_TARGET_MAX_ANGLE_DEGREE - 0) + J5_ZERO_US;
    } else {
        J5_TARGET_US = (J5_TARGET_ANGLE_DEGREE - J5_TARGET_MIN_ANGLE_DEGREE) * (J5_ZERO_US - J5_MIN_US) / (0 - J5_TARGET_MIN_ANGLE_DEGREE) + J5_MIN_US;
    }
}

void calculate_current_position() {
    ENCODER_4_CURRENT_PULSES = get_current_pulses(0);

    int16_t rotation = 0;

    rotation = (int16_t)ENCODER_4_CURRENT_PULSES - (int16_t)ENCODER_4_ZERO_POSITION;
	if (rotation > ENCODER_HALF_TOTAL_PULSES)
		rotation = -((0x3FFF) - rotation);
    if (rotation < -0x1FFF)
        rotation = rotation + 0x3FFF;

    ENCODER_4_CURRENT_NORM_PULSES = rotation;

    J4_CURRENT_ANGLE_DEGREE = ENCODER_DEGREE_IN_PULSE * ENCODER_4_CURRENT_NORM_PULSES;

    J4_CURRENT_STEPS = round(J4_CURRENT_ANGLE_DEGREE / J4_DEGREE_IN_STEP);
}
#endif

//// UART INPUT ////
bool checkPacket(uint8_t* packet) {
    uint8_t result = 0;
    uint16_t sum = 0;

    uint8_t original_result = packet[PACKET_SIZE - 1];
  
    for (uint8_t i = 0; i < (PACKET_SIZE - 1); i++)
        sum += packet[i];

    result = sum & CHECKSUM_BYTE;

    if (original_result == result)
        return true;

    return false;
}

void parsePacket(uint8_t* packet, TargetPosition* target_position_0, TargetPosition* target_position_1) {
    uint8_t i, j;
    for (i = 0, j = 1; i < 4; i++, j++) 
      target_position_0->b[i] = packet[j];

    for (i = 0, j = 5; i < 4; i++, j++) 
      target_position_1->b[i] = packet[j];
}

void on_uart_rx() {
    while (uart_is_readable(UART_ID)) {
        uint8_t c = uart_getc(UART_ID);

        if (c == HEAD_BYTE) {
            if (!first_time_header) {
                is_header = true;
                first_time_header = true;
                read_counter = 0;
            }
        }

        packet[read_counter] = c;
        read_counter++;

        if (read_counter >= PACKET_SIZE) {
            read_counter = 0;

            if (is_header) {
                if (checkPacket(packet)) {
                    parsePacket(packet, &tp_0, &tp_1);
                }
                is_header = false;
                first_time_header = false;
            }
        }
    }
}

//// MAIN LOOP ////
bool repeating_timer_callback(struct repeating_timer *t) {

#if CONTROLLER_ID == 0
    calculate_target_position();
    calculate_current_position();

    J0_STEPS_TO_GO = J0_TARGET_STEPS - J0_CURRENT_STEPS;
    J1_STEPS_TO_GO = J1_TARGET_STEPS - J1_CURRENT_STEPS;
    J1_STEPS_TO_GO = -1 * J1_STEPS_TO_GO;

    uint32_t ABS_J0_STEPS_TO_GO = J0_STEPS_TO_GO < 0 ? -1 * J0_STEPS_TO_GO : J0_STEPS_TO_GO;
    uint32_t ABS_J1_STEPS_TO_GO = J1_STEPS_TO_GO < 0 ? -1 * J1_STEPS_TO_GO : J1_STEPS_TO_GO;

    if (J0_PREVIOUS_TARGET_STEPS != J0_TARGET_STEPS) {
        J0_REACHED = false;
        J0_PREVIOUS_TARGET_STEPS = J0_TARGET_STEPS;
    }

    if (ABS_J0_STEPS_TO_GO == 0 && J0_REACHED == false) {
        J0_REACHED = true;
    }

    if (ABS_J0_STEPS_TO_GO > 5 && J0_REACHED == true)
        J0_REACHED == false;
            
    if (J0_PREVIOUS_TARGET_STEPS == J0_TARGET_STEPS) {
        if (J0_REACHED == true) {
                stop(0);
        }
        if (J0_REACHED == false) {
            move(0, J0_STEPS_TO_GO);
        }
    }

    if (J1_PREVIOUS_TARGET_STEPS != J1_TARGET_STEPS) {
        J1_REACHED = false;
        J1_PREVIOUS_TARGET_STEPS = J1_TARGET_STEPS;
    }

    if (ABS_J1_STEPS_TO_GO == 0 && J1_REACHED == false) {
        J1_REACHED = true;
    }

    if (ABS_J1_STEPS_TO_GO > 5 && J1_REACHED == true)
        J1_REACHED == false;
            
    if (J1_PREVIOUS_TARGET_STEPS == J1_TARGET_STEPS) {
        if (J1_REACHED == true) {
                stop_2(1);
        }
        if (J1_REACHED == false) {
            move(1, J1_STEPS_TO_GO);
        }
    }

#endif

#if CONTROLLER_ID == 1
    calculate_target_position();
    calculate_current_position();

    J2_STEPS_TO_GO = J2_TARGET_STEPS - J2_CURRENT_STEPS;
    J3_STEPS_TO_GO = J3_TARGET_STEPS - J3_CURRENT_STEPS;
    uint32_t ABS_J2_STEPS_TO_GO = J2_STEPS_TO_GO < 0 ? -1 * J2_STEPS_TO_GO : J2_STEPS_TO_GO;
    uint32_t ABS_J3_STEPS_TO_GO = J3_STEPS_TO_GO < 0 ? -1 * J3_STEPS_TO_GO : J3_STEPS_TO_GO;

    if (J2_PREVIOUS_TARGET_STEPS != J2_TARGET_STEPS) {
        J2_REACHED = false;
        J2_PREVIOUS_TARGET_STEPS = J2_TARGET_STEPS;
    }

    if (ABS_J2_STEPS_TO_GO == 0 && J2_REACHED == false) {
        J2_REACHED = true;
    }

    if (ABS_J2_STEPS_TO_GO > 5 && J2_REACHED == true)
        J2_REACHED == false;
            
    if (J2_PREVIOUS_TARGET_STEPS == J2_TARGET_STEPS) {
        if (J2_REACHED == true) {
                stop(0);
        }
        if (J2_REACHED == false) {
            move(0, J2_STEPS_TO_GO);
        }
    }

    if (J3_PREVIOUS_TARGET_STEPS != J3_TARGET_STEPS) {
        J3_REACHED = false;
        J3_PREVIOUS_TARGET_STEPS = J3_TARGET_STEPS;
    }

    if (ABS_J3_STEPS_TO_GO == 0 && J3_REACHED == false) {
        J3_REACHED = true;
    }

    if (ABS_J3_STEPS_TO_GO > 5 && J3_REACHED == true)
        J3_REACHED == false;
            
    if (J3_PREVIOUS_TARGET_STEPS == J3_TARGET_STEPS) {
        if (J3_REACHED == true) {
                stop(1);
        }
        if (J3_REACHED == false) {
            move(1, J3_STEPS_TO_GO);
        }
    }
#endif

#if CONTROLLER_ID == 2
    calculate_target_position();
    calculate_current_position();

    J4_STEPS_TO_GO = J4_TARGET_STEPS - J4_CURRENT_STEPS;
    uint32_t ABS_J4_STEPS_TO_GO = J4_STEPS_TO_GO < 0 ? -1 * J4_STEPS_TO_GO : J4_STEPS_TO_GO;

    if (J4_PREVIOUS_TARGET_STEPS != J4_TARGET_STEPS) {
        J4_REACHED = false;
        J4_PREVIOUS_TARGET_STEPS = J4_TARGET_STEPS;
    }

    if (ABS_J4_STEPS_TO_GO == 0 && J4_REACHED == false) {
        J4_REACHED = true;
    }

    if (ABS_J4_STEPS_TO_GO > 5 && J4_REACHED == true)
        J4_REACHED == false;
            
    if (J4_PREVIOUS_TARGET_STEPS == J4_TARGET_STEPS) {
        if (J4_REACHED == true) {
                stop(0);
        }
        if (J4_REACHED == false) {
            move(0, J4_STEPS_TO_GO);
        }
    }

    servo_write_microseconds(0, J5_TARGET_US);
    servo_write_microseconds(1, GRIPPER_TARGET_US);

#endif

#if DEBUG == 1
#if CONTROLLER_ID == 0
    printf("CPLS0: %-6d CPLS1: %-6d ", ENCODER_0_CURRENT_PULSES, ENCODER_1_CURRENT_PULSES);
    printf("NP0: %-6d NP1: %-6d ", ENCODER_0_CURRENT_NORM_PULSES, ENCODER_1_CURRENT_NORM_PULSES);
    printf("CS0: %-4d CS1: %-4d ", J0_CURRENT_STEPS, J1_CURRENT_STEPS);
    // printf("CA0: %-f CA1: %-f ", J0_CURRENT_ANGLE_DEGREE, J1_CURRENT_ANGLE_DEGREE);
    printf("TS0: %-4d TS1: %-4d ", J0_TARGET_STEPS, J1_TARGET_STEPS);
    printf("STG0: %-4d STG1: %-4d ", J0_STEPS_TO_GO, J1_STEPS_TO_GO);
    printf("PTS0: %-4d PTS0: %-4d ", J0_PREVIOUS_TARGET_STEPS, J1_PREVIOUS_TARGET_STEPS);
    printf("R0: %-4d R1: %-4d ", J0_REACHED, J1_REACHED);
#endif 
#if CONTROLLER_ID == 1
    printf("CPLS2: %-6d CPLS3: %-6d ", ENCODER_2_CURRENT_PULSES, ENCODER_3_CURRENT_PULSES);
    printf("NP2: %-6d NP3: %-6d ", ENCODER_2_CURRENT_NORM_PULSES, ENCODER_3_CURRENT_NORM_PULSES);
    printf("CS2: %-4d CS3: %-4d ", J2_CURRENT_STEPS, J3_CURRENT_STEPS);
    // printf("CA2: %-f CA3: %-f ", J2_CURRENT_ANGLE_DEGREE, J3_CURRENT_ANGLE_DEGREE);
    printf("TS2: %-4d TS3: %-4d ", J2_TARGET_STEPS, J3_TARGET_STEPS);
    printf("STG2: %-4d STG3: %-4d ", J2_STEPS_TO_GO, J3_STEPS_TO_GO);
    printf("PTS2: %-4d PTS3: %-4d ", J2_PREVIOUS_TARGET_STEPS, J3_PREVIOUS_TARGET_STEPS);
    printf("R2: %-4d R3: %-4d ", J2_REACHED, J3_REACHED);
#endif

#if CONTROLLER_ID == 2
    printf("CPLS4: %-6d ", ENCODER_4_CURRENT_PULSES);
    printf("NP4: %-6d ", ENCODER_4_CURRENT_NORM_PULSES);
    printf("CS4: %-4d ", J4_CURRENT_STEPS);
    // printf("CA4: %-f ", J4_CURRENT_ANGLE_DEGREE);
    printf("TS4: %-4d ", J4_TARGET_STEPS);
    printf("STG4: %-4d ", J4_STEPS_TO_GO);
    printf("PTS4: %-4d ", J4_PREVIOUS_TARGET_STEPS);
    printf("R4: %-4d ", J4_REACHED);
    printf("TA5: %-4f ", J5_TARGET_ANGLE_DEGREE);
    printf("TUS5: %-4d ", J5_TARGET_US);

#endif
    printf("\n");
#endif
    return true;
}

int main() {
    // UART START
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    // DEBUG UART
#if DEBUG == 1
    stdio_init_all();
#endif

#if CONTROLLER_ID == 0
    // STEPPER START
    stepper_driver_init();
    stepper_0_init();
    stepper_1_init();
    // ENCODERS START
    as5048a_init_device();
#endif
#if CONTROLLER_ID == 1
    // STEPPER START
    stepper_driver_init();
    stepper_0_init();
    stepper_1_init();
    // ENCODERS START
    as5048a_init_device();
#endif
#if CONTROLLER_ID == 2
    // STEPPER START
    stepper_driver_init();
    stepper_0_init();
    // SERVO & GRIPPER JOINT
    servo_driver_init();
    servo_0_init(J5_SERVO_GPIO_PIN);
    servo_1_init(GRIPPER_SERVO_GPIO_PIN);
    servo_attach(0);
    servo_attach(1);
    // ENCODERS START
    as5048a_init_device();
#endif

    // LOOP START
    struct repeating_timer timer;
    add_repeating_timer_ms(-1 * (1 / MAIN_LOOP_RATE_HZ * 1000), repeating_timer_callback, NULL, &timer);

    while (true) {
    }
}
