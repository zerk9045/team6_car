# This is a basic CMakeLists.txt file for the config directory
# Add the source files in this directory to a library
add_library(config_library STATIC pin_config.h)

# Specify the include directories
target_include_directories(config_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})