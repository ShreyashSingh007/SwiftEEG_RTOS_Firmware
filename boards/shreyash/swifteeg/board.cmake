# ST-Link V2 + OpenOCD. See openocd/swifteeg.cfg for the probe/target config.
board_runner_args(openocd "--config=${CMAKE_CURRENT_LIST_DIR}/../../../openocd/swifteeg.cfg")
board_runner_args(openocd "--use-elf")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
