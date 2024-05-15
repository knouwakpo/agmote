cmake_minimum_required(VERSION 2.6)

project(spidev-test)
set(CMAKE_CXX_FLAGS "-g -Wall")
set(CMAKE_C_FLAGS "-g -Wall")

include_directories(../src/)

add_executable(
     spi-testc spidev-testc.c
)
target_link_libraries( spi-testc "/home/roccflume/spidev/spidev-lib-master/build/libspidev-lib.a")

add_executable(spi-testcpp spidev-testcpp.cc)
target_link_libraries( spi-testcpp "/home/roccflume/spidev/spidev-lib-master/build/libspidev-lib++.a" )

