add_executable(
        magnetometer_test
        magnetometer_test.c
)

target_link_libraries(
        magnetometer_test
        hardware_i2c
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

target_include_directories(
        magnetometer_test
        PRIVATE ../config
)

pico_enable_stdio_usb(magnetometer_test 1)
pico_add_extra_outputs(magnetometer_test)