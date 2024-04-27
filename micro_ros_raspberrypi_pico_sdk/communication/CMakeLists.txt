# This is a basic CMakeLists.txt file for the communication directory
# Add the source files in this directory to a library
add_library(communication_library STATIC pico_uart_transport.cpp)

# Specify the include directories
target_include_directories(communication_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(communication_library PUBLIC
        pico_stdlib hardware_uart
)