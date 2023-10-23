add_executable(
        ultrasonic_sensor
        ultrasonic_sensor.c
        )

target_link_libraries(
        ultrasonic_sensor
        hardware_adc
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )

target_include_directories(ultrasonic_sensor 
        PRIVATE ../config
        )

pico_enable_stdio_usb(ultrasonic_sensor 1)
pico_add_extra_outputs(ultrasonic_sensor)