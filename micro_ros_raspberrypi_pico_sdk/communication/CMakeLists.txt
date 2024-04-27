# This is a basic CMakeLists.txt file for the communication directory
# Add the source files in this directory to a library
add_library(communication_library STATIC communication_file1.cpp communication_file2.cpp)

# Specify the include directories
target_include_directories(communication_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})