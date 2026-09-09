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

/*
 * Largest notification payload the current connection will carry, or 0 when
 * nobody is connected. Batches are sized against this: a notification bigger
 * than the negotiated MTU is silently dropped by the stack.
 */
uint16_t ble_transport_max_payload(void);

/*
 * Sends on the Event characteristic - command replies and telemetry. Kept
 * off the Stream characteristic so a host can subscribe to replies without
 * having to receive sample data.
 */
int ble_transport_send_event(const void *data, uint16_t len);

/*
 * Installed by the command layer. Called from the BLE thread whenever the
 * host writes to the Control characteristic; the bytes are protocol frames,
 * identical to what arrives over USB.
 */
typedef void (*ble_control_cb_t)(const uint8_t *data, uint16_t len);
void ble_transport_set_control_handler(ble_control_cb_t cb);

#endif /* SWIFTEEG_TRANSPORT_BLE_H */
