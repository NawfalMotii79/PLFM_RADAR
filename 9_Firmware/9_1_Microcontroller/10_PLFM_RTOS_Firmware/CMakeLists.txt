cmake_minimum_required(VERSION 3.20)
project(plfm_rtos_firmware C)

add_library(plfm_rtos STATIC
    src/plfm_bootloader.c
    src/plfm_config.c
    src/plfm_dsp.c
    src/plfm_protocol.c
    src/plfm_tasks.c
)

target_include_directories(plfm_rtos PUBLIC include)
target_compile_features(plfm_rtos PUBLIC c_std_11)
target_compile_options(plfm_rtos PRIVATE -Wall -Wextra -Wpedantic)

# Hook this target into your existing STM32Cube/CMake build.
add_executable(plfm_rtos_app src/main_plfm_rtos.c)
target_link_libraries(plfm_rtos_app PRIVATE plfm_rtos m)
