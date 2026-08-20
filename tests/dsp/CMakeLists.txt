cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
list(APPEND DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dsp_test LANGUAGES C)

# Covers both pure modules: the DSP primitives and the SPSC ring buffer.
target_sources(app PRIVATE
  src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/dsp/dsp.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/sys/ringbuf.c
)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/dsp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/sys
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(app PRIVATE -Wall -Wextra -Werror)
