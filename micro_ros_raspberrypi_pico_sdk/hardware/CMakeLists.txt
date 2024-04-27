# This is a basic CMakeLists.txt file for the hardware directory
# Add the source files in this directory to a library
add_library(hardware_library STATIC IRSensor.cpp Motor.cpp Servo.cpp)

# Specify the include directories
target_include_directories(hardware_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hardware_library PUBLIC pico_stdlib hardware_pwm hardware_adc hardware_gpio hardware_spi hardware_i2c hardware_timer hardware_uart hardware_dma hardware_divider hardware_clocks hardware_irq hardware_sync hardware_watchdog hardware_xosc hardware_pll hardware_vreg hardware_rtc hardware_structs hardware_claim)