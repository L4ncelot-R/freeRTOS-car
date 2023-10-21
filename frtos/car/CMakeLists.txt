add_library(
        car
        car.h
)

target_link_libraries(
        car
        hardware_adc
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

target_include_directories(car
        PRIVATE ../config
)