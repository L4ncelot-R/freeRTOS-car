include(FreeRTOS_Kernel_import.cmake)

add_subdirectory(motor)
add_subdirectory(line_sensor)
add_subdirectory(car)
add_subdirectory(ultrasonic_sensor)
add_subdirectory(magnetometer)
add_subdirectory(frontend)

add_executable(rtos_car rtos_car.c)

target_compile_definitions(rtos_car PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        PING_USE_SOCKETS=1
)
target_include_directories(rtos_car PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/config
        ${CMAKE_CURRENT_LIST_DIR}/motor
        ${CMAKE_CURRENT_LIST_DIR}/line_sensor
        ${CMAKE_CURRENT_LIST_DIR}/ultrasonic_sensor
        ${CMAKE_CURRENT_LIST_DIR}/frontend
)
target_link_libraries(rtos_car
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        hardware_adc
        hardware_pwm
        hardware_i2c
)
pico_enable_stdio_usb(rtos_car 1)
pico_add_extra_outputs(rtos_car)
