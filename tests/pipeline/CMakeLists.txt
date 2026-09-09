cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
list(APPEND DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pipeline_test LANGUAGES C)

# The chain under test: frame decode (header-only) plus the DSP primitives,
# composed here exactly as src/pipeline/pipeline.c composes them.
target_sources(app PRIVATE
  src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/dsp/dsp.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/pipeline/chain.c
)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/dsp
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(app PRIVATE -Wall -Wextra -Werror)
