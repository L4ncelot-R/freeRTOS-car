add_executable(
        barcode_sensor_test
        barcode_sensor_test.c
)

target_link_libraries(
        barcode_sensor_test
        hardware_adc
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

target_include_directories(barcode_sensor_test
        PRIVATE ../config
        ../line_sensor
)

pico_enable_stdio_usb(barcode_sensor_test 1)
pico_add_extra_outputs(barcode_sensor_test)