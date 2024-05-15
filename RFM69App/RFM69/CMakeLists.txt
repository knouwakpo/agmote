file(GLOB FILES *.c *.h)
add_library(RFM69 ${FILES})
target_link_libraries(RFM69 pigpio )
target_include_directories(RFM69 PUBLIC "${PROJECT_SOURCE_DIR}/RFM69")
