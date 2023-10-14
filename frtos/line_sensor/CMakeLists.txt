add_library(line_sensor
        line_sensor.c
        line_sensor.h
        Config.h
)

target_link_libraries(line_sensor
        hardware_adc
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

target_include_directories(line_sensor
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")