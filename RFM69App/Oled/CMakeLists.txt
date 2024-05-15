file(GLOB FILES *.c *.h)
add_library(OLED ${FILES})
target_link_libraries(OLED ssd1306_i2c )
target_include_directories(OLED PUBLIC "${PROJECT_SOURCE_DIR}/Oled")
