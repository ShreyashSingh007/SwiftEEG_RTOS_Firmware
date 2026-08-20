/*
 * BLE transport - custom GATT service.
 *
 * Three characteristics, mirroring the binary protocol's message types:
 *   Control  (write)   host -> device commands
 *   Stream   (notify)  sample data
 *   Event    (notify)  asynchronous status and telemetry
 *
 * The wire format is identical to USB; only the framing below differs. That
 * is deliberate - the host library is written once and works on both links.
 *
 * Latency floor is platform-dependent and NOT something firmware can fix:
 * Android/Windows/Linux permit a 7.5 ms connection interval, but iOS
 * enforces a 15 ms minimum, so sub-10 ms end-to-end is unreachable there.
 */
#ifndef SWIFTEEG_TRANSPORT_BLE_H
#define SWIFTEEG_TRANSPORT_BLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Enables the controller, registers the service and starts advertising. */
int ble_transport_init(void);

/* True while a central is connected and subscribed to the stream. */
bool ble_transport_is_streaming(void);

/* Queues a stream notification. -ENOTCONN if nobody is subscribed. */
int ble_transport_send_stream(const void *data, uint16_t len);

#endif /* SWIFTEEG_TRANSPORT_BLE_H */
