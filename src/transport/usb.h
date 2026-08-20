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

/* Brings up the USB device stack. Returns 0 on success, negative errno. */
int usb_transport_init(void);

/* True once the host has opened the CDC ACM port (DTR asserted). */
bool usb_transport_is_connected(void);

#endif /* SWIFTEEG_TRANSPORT_USB_H */
