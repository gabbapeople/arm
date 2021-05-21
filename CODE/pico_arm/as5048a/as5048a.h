#ifndef AS5048A_H
#define AS5048A_H

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define SPI_0_CS_GPIO_PIN 2
#define SPI_1_CS_GPIO_PIN 3

#define SPI_0_MISO_GPIO_PIN 16
#define SPI_0_MOSI_GPIO_PIN 18
#define SPI_0_SCK_GPIO_PIN 19

#define SPI_1_MISO_GPIO_PIN 8
#define SPI_1_MOSI_GPIO_PIN 15
#define SPI_1_SCK_GPIO_PIN 14

#define SPI_PORT_0 spi0
#define SPI_PORT_1 spi1

#define AS5048_DEBUG 0

// IMPLEMENTED
#define AS5048A_ANGLE 0x3FFF

// NOT IMPLEMENTED
#define AS5048A_CLEAR_ERROR_FLAG 0x0001
#define AS5048A_PROGRAMMING_CONTROL 0x0003
#define AS5048A_OTP_REGISTER_ZERO_POS_HIGH 0x0016
#define AS5048A_OTP_REGISTER_ZERO_POS_LOW 0x0017
#define AS5048A_DIAG_AGC 0x3FFD
#define AS5048A_MAGNITUDE 0x3FFE

#define AS5048A_AGC_FLAG 0xFF
#define AS5048A_ERROR_PARITY_FLAG 0x04
#define AS5048A_ERROR_COMMAND_INVALID_FLAG 0x02
#define AS5048A_ERROR_FRAMING_FLAG 0x01

#define AS5048A_DIAG_COMP_HIGH 0x2000
#define AS5048A_DIAG_COMP_LOW 0x1000
#define AS5048A_DIAG_COF 0x0800
#define AS5048A_DIAG_OCF 0x0400

#define AS5048A_RESOLUTION 16384.0

void as5048a_init_device();
static uint8_t spi_calc_even_parity(uint16_t value);

static uint16_t as5048a_read16(uint8_t device_id, uint16_t reg);
uint16_t as5048a_get_raw_rotation(uint8_t device_id);
double as5048a_get_angle_in_rad(uint8_t device_id);
uint16_t as5048a_get_simple_avr_raw_rotation(uint8_t device_id, uint16_t n);

#endif // AS5048A_H
