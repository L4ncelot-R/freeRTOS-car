add_executable(motor_test motor_test.c)

target_link_libraries(motor_test
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        hardware_pwm
)

target_include_directories(motor_test PRIVATE
        ../config
)

pico_enable_stdio_usb(motor_test 1)
pico_add_extra_outputs(motor_test)