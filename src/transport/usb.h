/*
 * USB transport - CDC ACM.
 *
 * Wired link for the binary protocol. Enumerates as a virtual serial port,
 * so no driver install is needed on Windows 10+, macOS or Linux. This is the
 * only transport fast enough for the top sample rates: 8ch @ 16 kSPS is about
 * 432 kB/s, well beyond BLE's practical ~1.4 Mbps shared ceiling.
 *
 * Note there is no USB path on iOS - that platform is BLE only.
 */
#ifndef SWIFTEEG_TRANSPORT_USB_H
#define SWIFTEEG_TRANSPORT_USB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Brings up the USB device stack. Returns 0 on success, negative errno. */
int usb_transport_init(void);

/* True once the host has opened the CDC ACM port (DTR asserted). */
bool usb_transport_is_connected(void);

/*
 * Queue bytes for transmission. Returns how many were accepted, which is
 * less than `len` if the buffer is full.
 *
 * Never blocks. The acquisition path must not be able to stall behind a host
 * that has stopped reading, so a slow reader loses data rather than backing
 * pressure up into the pipeline. Dropped bytes are counted.
 */
size_t usb_transport_write(const uint8_t *buf, size_t len);

/* Bytes discarded because the transmit buffer was full. */
uint32_t usb_transport_dropped(void);

/*
 * Take up to `len` received bytes. Returns how many were copied, which is 0
 * when nothing has arrived. Never blocks.
 */
size_t usb_transport_read(uint8_t *buf, size_t len);

#endif /* SWIFTEEG_TRANSPORT_USB_H */
