file(GLOB FILES *.c *.h)
add_library(RFM69Config ${FILES})
target_link_libraries(RFM69Config RFM69 pigpio)
target_include_directories(RFM69Config PUBLIC "${PROJECT_SOURCE_DIR}/RFM69Config")
