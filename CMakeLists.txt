cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board lives in this repo.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(swifteeg_fw LANGUAGES C)

target_sources(app PRIVATE
  src/main.c
  src/board/leds.c
  src/board/supply.c
  src/afe/ads1299.c
  src/timebase/timebase.c
  src/pipeline/capture.c
  src/transport/usb.c
  src/transport/ble.c
)

target_include_directories(app PRIVATE src)

# Treat warnings as errors. This is a medical-adjacent signal chain; a
# warning here is a bug we have not diagnosed yet.
target_compile_options(app PRIVATE -Wall -Wextra -Werror)
