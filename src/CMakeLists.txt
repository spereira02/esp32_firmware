# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS "main.c" "esp32_serial_transport.c"
                       INCLUDE_DIRS "."
                       REQUIRES micro_ros_espidf_component)
