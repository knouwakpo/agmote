cmake_minimum_required(VERSION 2.6)



project(rfm69app C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(${PROJECT_NAME} main.c)
add_subdirectory("$ENV{HOME}/RFM69App/RFM69")
add_subdirectory("$ENV{HOME}/RFM69App/RFM69Config")
add_subdirectory("$ENV{HOME}/RFM69App/Oled")

target_link_libraries(${PROJECT_NAME} RFM69 RFM69Config Threads::Threads pigpio OLED)

message(PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

target_include_directories(${PROJECT_NAME} PUBLIC 
    "${PROJECT_BINARY_DIR}")


