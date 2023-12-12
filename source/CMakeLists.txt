cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico-test)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(pico_ultrasonic
        main.c
        usr_i2c.c
        ch_lib.c
        ch_drv_ch101.c
        ch_drv_ch201.c
        ch_com.c
        usr_bsp.c
        usr_def.c
        ch201_gprmt_fw.c
        ch101_gpr_fw.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(pico_ultrasonic pico_stdlib hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_ultrasonic 1)
pico_enable_stdio_uart(pico_ultrasonic 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_ultrasonic)