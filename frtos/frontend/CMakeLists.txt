add_executable(
        frontend
        frontend.c
        )

message("Running makefsdata C script")
execute_process(COMMAND
    gcc makefsdata.c -o makefsdata.exe
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)
message("Generating htmldata.c")
execute_process(COMMAND
    ./makefsdata.exe
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(
        frontend
        hardware_adc
        pico_stdlib
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        pico_cyw43_arch_lwip_sys_freertos
        pico_lwip_http
        )

target_include_directories(frontend PRIVATE
        ../config
        ${CMAKE_CURRENT_LIST_DIR}
        )

pico_enable_stdio_usb(frontend 1)
pico_add_extra_outputs(frontend)