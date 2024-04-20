cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(my_project_name)

pico_sdk_init()

add_executable(${PROJECT_NAME}
        main.cpp
        communication/pico_uart_transport.c
        hardware/Motor.cpp
        hardware/Servo.cpp
)

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        micro_ros
        hardware_pwm
)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/config
        ${CMAKE_CURRENT_LIST_DIR}/communication
        ${CMAKE_CURRENT_LIST_DIR}/hardware
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(${PROJECT_NAME})
