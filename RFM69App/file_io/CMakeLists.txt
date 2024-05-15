file(GLOB FILES *.c *.h)
add_library(file_io ${FILES})
target_link_libraries(file_io RFM69 pigpio)
target_include_directories(file_io PUBLIC "${PROJECT_SOURCE_DIR}/file_io")
