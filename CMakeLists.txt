cmake_minimum_required(VERSION 3.13)
# project(as7265x_pico C)

add_library(as7265x_pico INTERFACE)

target_sources(as7265x_pico INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/src/as7265x.c
)

target_include_directories(as7265x_pico INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/include)

# Link hardware_i2c for the Pico SDK
target_link_libraries(as7265x_pico INTERFACE 
    pico_stdlib 
    hardware_i2c
)