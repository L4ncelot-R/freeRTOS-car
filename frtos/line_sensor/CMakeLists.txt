add_executable(
        line_sensor_test
        line_sensor_test.c
)

target_link_libraries(
        line_sensor_test
        hardware_adc
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

target_include_directories(line_sensor_test
        PRIVATE ../config
        ../car
        ../ultrasonic_sensor
)

pico_enable_stdio_usb(line_sensor_test 1)
pico_add_extra_outputs(line_sensor_test)