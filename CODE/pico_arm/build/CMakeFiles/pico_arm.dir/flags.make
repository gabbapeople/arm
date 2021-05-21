# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# compile ASM with /usr/bin/arm-none-eabi-gcc
# compile C with /usr/bin/arm-none-eabi-gcc
# compile CXX with /usr/bin/arm-none-eabi-g++
ASM_FLAGS = -march=armv6-m -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG   -ffunction-sections -fdata-sections

ASM_DEFINES = -DCFG_TUSB_DEBUG=0 -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DPICO_BIT_OPS_PICO=1 -DPICO_BOARD=\"pico\" -DPICO_BOOT2_NAME=\"boot2_w25q080\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DIVIDER_HARDWARE=1 -DPICO_DOUBLE_PICO=1 -DPICO_FLOAT_PICO=1 -DPICO_INT64_OPS_PICO=1 -DPICO_MEM_OPS_PICO=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_PRINTF_PICO=1 -DPICO_STDIO_USB=1 -DPICO_TARGET_NAME=\"pico_arm\" -DPICO_USE_BLOCKED_RAM=0

ASM_INCLUDES = -I/home/ubuntu/ARM/pico_arm/build -I/home/ubuntu/ARM/pico_arm/stepper -I/home/ubuntu/ARM/pico_arm/as5048a -I/home/ubuntu/ARM/pico_arm/servo -I/home/ubuntu/pico-sdk/src/common/pico_stdlib/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_gpio/include -I/home/ubuntu/pico-sdk/src/common/pico_base/include -I/home/ubuntu/ARM/pico_arm/build/generated/pico_base -I/home/ubuntu/pico-sdk/src/boards/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_platform/include -I/home/ubuntu/pico-sdk/src/rp2040/hardware_regs/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_base/include -I/home/ubuntu/pico-sdk/src/rp2040/hardware_structs/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_claim/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_sync/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_uart/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_divider/include -I/home/ubuntu/pico-sdk/src/common/pico_time/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_timer/include -I/home/ubuntu/pico-sdk/src/common/pico_sync/include -I/home/ubuntu/pico-sdk/src/common/pico_util/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_runtime/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_clocks/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_resets/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_watchdog/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_xosc/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_pll/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_vreg/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_irq/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_printf/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_bootrom/include -I/home/ubuntu/pico-sdk/src/common/pico_bit_ops/include -I/home/ubuntu/pico-sdk/src/common/pico_divider/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_double/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_int64_ops/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_float/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_malloc/include -I/home/ubuntu/pico-sdk/src/common/pico_binary_info/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_stdio/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_stdio_usb/include -I/home/ubuntu/pico-sdk/lib/tinyusb/src -I/home/ubuntu/pico-sdk/lib/tinyusb/src/common -I/home/ubuntu/pico-sdk/lib/tinyusb/hw -I/home/ubuntu/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_pio/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_spi/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_adc/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_pwm/include 

C_FLAGS = -march=armv6-m -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG   -ffunction-sections -fdata-sections

C_DEFINES = -DCFG_TUSB_DEBUG=0 -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DPICO_BIT_OPS_PICO=1 -DPICO_BOARD=\"pico\" -DPICO_BOOT2_NAME=\"boot2_w25q080\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DIVIDER_HARDWARE=1 -DPICO_DOUBLE_PICO=1 -DPICO_FLOAT_PICO=1 -DPICO_INT64_OPS_PICO=1 -DPICO_MEM_OPS_PICO=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_PRINTF_PICO=1 -DPICO_STDIO_USB=1 -DPICO_TARGET_NAME=\"pico_arm\" -DPICO_USE_BLOCKED_RAM=0

C_INCLUDES = -I/home/ubuntu/ARM/pico_arm/build -I/home/ubuntu/ARM/pico_arm/stepper -I/home/ubuntu/ARM/pico_arm/as5048a -I/home/ubuntu/ARM/pico_arm/servo -I/home/ubuntu/pico-sdk/src/common/pico_stdlib/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_gpio/include -I/home/ubuntu/pico-sdk/src/common/pico_base/include -I/home/ubuntu/ARM/pico_arm/build/generated/pico_base -I/home/ubuntu/pico-sdk/src/boards/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_platform/include -I/home/ubuntu/pico-sdk/src/rp2040/hardware_regs/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_base/include -I/home/ubuntu/pico-sdk/src/rp2040/hardware_structs/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_claim/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_sync/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_uart/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_divider/include -I/home/ubuntu/pico-sdk/src/common/pico_time/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_timer/include -I/home/ubuntu/pico-sdk/src/common/pico_sync/include -I/home/ubuntu/pico-sdk/src/common/pico_util/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_runtime/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_clocks/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_resets/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_watchdog/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_xosc/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_pll/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_vreg/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_irq/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_printf/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_bootrom/include -I/home/ubuntu/pico-sdk/src/common/pico_bit_ops/include -I/home/ubuntu/pico-sdk/src/common/pico_divider/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_double/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_int64_ops/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_float/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_malloc/include -I/home/ubuntu/pico-sdk/src/common/pico_binary_info/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_stdio/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_stdio_usb/include -I/home/ubuntu/pico-sdk/lib/tinyusb/src -I/home/ubuntu/pico-sdk/lib/tinyusb/src/common -I/home/ubuntu/pico-sdk/lib/tinyusb/hw -I/home/ubuntu/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_pio/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_spi/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_adc/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_pwm/include 

CXX_FLAGS = -march=armv6-m -mcpu=cortex-m0plus -mthumb -O3 -DNDEBUG   -ffunction-sections -fdata-sections -fno-exceptions -fno-unwind-tables -fno-rtti -fno-use-cxa-atexit

CXX_DEFINES = -DCFG_TUSB_DEBUG=0 -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DPICO_BIT_OPS_PICO=1 -DPICO_BOARD=\"pico\" -DPICO_BOOT2_NAME=\"boot2_w25q080\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Release\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DIVIDER_HARDWARE=1 -DPICO_DOUBLE_PICO=1 -DPICO_FLOAT_PICO=1 -DPICO_INT64_OPS_PICO=1 -DPICO_MEM_OPS_PICO=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_PRINTF_PICO=1 -DPICO_STDIO_USB=1 -DPICO_TARGET_NAME=\"pico_arm\" -DPICO_USE_BLOCKED_RAM=0

CXX_INCLUDES = -I/home/ubuntu/ARM/pico_arm/build -I/home/ubuntu/ARM/pico_arm/stepper -I/home/ubuntu/ARM/pico_arm/as5048a -I/home/ubuntu/ARM/pico_arm/servo -I/home/ubuntu/pico-sdk/src/common/pico_stdlib/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_gpio/include -I/home/ubuntu/pico-sdk/src/common/pico_base/include -I/home/ubuntu/ARM/pico_arm/build/generated/pico_base -I/home/ubuntu/pico-sdk/src/boards/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_platform/include -I/home/ubuntu/pico-sdk/src/rp2040/hardware_regs/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_base/include -I/home/ubuntu/pico-sdk/src/rp2040/hardware_structs/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_claim/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_sync/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_uart/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_divider/include -I/home/ubuntu/pico-sdk/src/common/pico_time/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_timer/include -I/home/ubuntu/pico-sdk/src/common/pico_sync/include -I/home/ubuntu/pico-sdk/src/common/pico_util/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_runtime/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_clocks/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_resets/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_watchdog/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_xosc/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_pll/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_vreg/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_irq/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_printf/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_bootrom/include -I/home/ubuntu/pico-sdk/src/common/pico_bit_ops/include -I/home/ubuntu/pico-sdk/src/common/pico_divider/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_double/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_int64_ops/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_float/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_malloc/include -I/home/ubuntu/pico-sdk/src/common/pico_binary_info/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_stdio/include -I/home/ubuntu/pico-sdk/src/rp2_common/pico_stdio_usb/include -I/home/ubuntu/pico-sdk/lib/tinyusb/src -I/home/ubuntu/pico-sdk/lib/tinyusb/src/common -I/home/ubuntu/pico-sdk/lib/tinyusb/hw -I/home/ubuntu/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_pio/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_spi/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_adc/include -I/home/ubuntu/pico-sdk/src/rp2_common/hardware_pwm/include 

