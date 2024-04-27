# This is a basic CMakeLists.txt file for the hardware directory
# Add the source files in this directory to a library
add_library(hardware_library STATIC hardware_file1.cpp hardware_file2.cpp)

# Specify the include directories
target_include_directories(hardware_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})