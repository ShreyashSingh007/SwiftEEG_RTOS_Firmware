/*
 * Supply rail measurement.
 *
 * The board has no fuel gauge and VDDH sits on the regulated rail, not the
 * cell, so true battery voltage is not observable until a divider jumper is
 * fitted to a free AIN pin.
 *
 * What IS observable is VDD itself: the nRF52840 SAADC can sample its own
 * supply. That gives the actual LP5907 output, which is useful on its own and
 * is the only brownout warning available until the divider exists - once the
 * cell sags below the regulator's dropout, VDD follows it down.
 */
#ifndef SWIFTEEG_BOARD_SUPPLY_H
#define SWIFTEEG_BOARD_SUPPLY_H

/* Reads VDD in millivolts. Negative errno on failure. */
int supply_read_vdd_mv(void);

/*
 * Samples VDD at three SAADC gains and logs each.
 *
 * Diagnostic, not routine. Agreement across gains proves the reading is real;
 * saturation on the narrower ranges would prove the rail is higher than
 * reported and the scaling is at fault. Settles "is the supply low, or is the
 * ADC lying" without a multimeter.
 */
void supply_selftest(void);

/*
 * Logs POWER.USBREGSTATUS. The nRF52840 keeps its USB device controller
 * disabled until both VBUSDETECT and OUTPUTRDY are set, so this says whether
 * a USB failure is the cable, the board, or the firmware.
 */
void supply_log_usb_status(void);

#endif /* SWIFTEEG_BOARD_SUPPLY_H */
