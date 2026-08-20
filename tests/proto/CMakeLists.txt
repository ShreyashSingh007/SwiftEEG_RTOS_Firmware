cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
list(APPEND DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(proto_test LANGUAGES C)

target_sources(app PRIVATE
  src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/proto/proto.c
)

# proto.h lives with the source; golden_vectors.h is generated next to us.
target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/proto
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(app PRIVATE -Wall -Wextra -Werror)
