add_executable(
        map_test
        map_test.c
)

target_link_libraries(
        map_test
        hardware_adc
        pico_stdlib
        pico_rand
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

target_include_directories(map_test
        PRIVATE ../config
        ../car
)

pico_enable_stdio_usb(map_test 1)
pico_add_extra_outputs(map_test)