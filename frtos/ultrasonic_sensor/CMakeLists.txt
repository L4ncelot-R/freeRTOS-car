add_executable(
        ultrasonic_sensor
        ultrasonic_sensor.c
        )

target_link_libraries(
        ultrasonic_sensor
        hardware_adc
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        hardware_i2c
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        hardware_pwm
        )

target_include_directories(ultrasonic_sensor 
        PRIVATE ../config
        ../motor
        )

pico_enable_stdio_usb(ultrasonic_sensor 1)
pico_add_extra_outputs(ultrasonic_sensor)