file(GLOB FILES *.c *.h)
add_library(SPIDEV ${FILES})
target_include_directories(SPIDEV PUBLIC "${PROJECT_SOURCE_DIR}/spidev")
