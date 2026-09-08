cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
list(APPEND DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(timebase_test LANGUAGES C)

# The wrap maths is header-only, so there is no source to compile here - the
# include path points at the real tree so the test exercises shipped code
# rather than a copy.
target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

target_compile_options(app PRIVATE -Wall -Wextra -Werror)
