cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_arm)

pico_sdk_init()

add_executable(pico_arm)

pico_generate_pio_header(pico_arm ${CMAKE_CURRENT_LIST_DIR}/stepper/stepper.pio)
pico_generate_pio_header(pico_arm ${CMAKE_CURRENT_LIST_DIR}/servo/servo.pio)

target_include_directories(pico_arm PRIVATE
    stepper
    as5048a
    servo)

target_sources(pico_arm PRIVATE
    pico_arm.c
    stepper/stepper.c
    as5048a/as5048a.c
    servo/servo.c)

target_link_libraries(pico_arm PRIVATE
    pico_stdlib
    hardware_pio
    hardware_spi
    hardware_adc
    hardware_pwm
    hardware_uart)

pico_enable_stdio_usb(pico_arm 1)
pico_enable_stdio_uart(pico_arm 0)

pico_add_extra_outputs(pico_arm)
