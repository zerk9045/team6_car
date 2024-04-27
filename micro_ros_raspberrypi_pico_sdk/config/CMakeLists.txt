# This is a basic CMakeLists.txt file for the config directory
# Add the source files in this directory to a library
add_library(config_library STATIC config_file1.cpp config_file2.cpp)

# Specify the include directories
target_include_directories(config_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})