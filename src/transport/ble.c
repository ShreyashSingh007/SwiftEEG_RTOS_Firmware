#include "ble.h"

/*
 * Everything below needs the Bluetooth subsystem. When CONFIG_BT is off the
 * module compiles to stubs, which allows a BLE-less diagnostic build.
 *
 * That build exists for a concrete reason: once MPSL and the radio are
 * running they hold the bus for timing, and RTT reads over the ST-Link HLA
 * transport become unreliable enough to drop the debug connection entirely.
 * Turning BLE off is currently the only way to get trustworthy logs.
 */
#if defined(CONFIG_BT)

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Custom 128-bit UUIDs, first word incrementing per characteristic.
 *
 * The final field is 48 bits and MUST carry a ULL suffix: the encoding macro
 * shifts it right by 32, and an unsuffixed literal is a 32-bit int, which is
 * undefined behaviour (and a build error under -Werror). The node value spells
 * "SWIFTE" in ASCII.
 */
#define SWIFTEEG_UUID_NODE 0x535749465445ULL
#define SWIFTEEG_UUID_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x57724501, 0x4700, 0x4000, 0x8000, SWIFTEEG_UUID_NODE)
#define SWIFTEEG_UUID_CONTROL_VAL \
	BT_UUID_128_ENCODE(0x57724502, 0x4700, 0x4000, 0x8000, SWIFTEEG_UUID_NODE)
#define SWIFTEEG_UUID_STREAM_VAL \
	BT_UUID_128_ENCODE(0x57724503, 0x4700, 0x4000, 0x8000, SWIFTEEG_UUID_NODE)
#define SWIFTEEG_UUID_EVENT_VAL \
	BT_UUID_128_ENCODE(0x57724504, 0x4700, 0x4000, 0x8000, SWIFTEEG_UUID_NODE)

static const struct bt_uuid_128 uuid_service =
	BT_UUID_INIT_128(SWIFTEEG_UUID_SERVICE_VAL);
static const struct bt_uuid_128 uuid_control =
	BT_UUID_INIT_128(SWIFTEEG_UUID_CONTROL_VAL);
static const struct bt_uuid_128 uuid_stream =
	BT_UUID_INIT_128(SWIFTEEG_UUID_STREAM_VAL);
static const struct bt_uuid_128 uuid_event =
	BT_UUID_INIT_128(SWIFTEEG_UUID_EVENT_VAL);

static struct bt_conn *current_conn;
static bool stream_subscribed;

/* --- Control characteristic ------------------------------------------- */

static ssize_t control_write(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     const void *buf, uint16_t len,
			     uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	/*
	 * M1 stub: the protocol codec lands in M2. Log so the link can be
	 * exercised end to end from a host before any of it is decoded.
	 */
	LOG_INF("control write, %u bytes (first 0x%02x)", len,
		len ? ((const uint8_t *)buf)[0] : 0);

	return len;
}

static void stream_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	stream_subscribed = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("stream notifications %s",
		stream_subscribed ? "enabled" : "disabled");
}

static void event_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	LOG_INF("event notifications %s",
		(value == BT_GATT_CCC_NOTIFY) ? "enabled" : "disabled");
}

/*
 * Attribute layout. The stream value attribute is index 2 in this table,
 * which is what bt_gatt_notify() is pointed at below.
 */
BT_GATT_SERVICE_DEFINE(swifteeg_svc,
	BT_GATT_PRIMARY_SERVICE(&uuid_service),

	BT_GATT_CHARACTERISTIC(&uuid_control.uuid,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE,
			       NULL, control_write, NULL),

	BT_GATT_CHARACTERISTIC(&uuid_stream.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(stream_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(&uuid_event.uuid,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(event_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* --- Connection handling ---------------------------------------------- */

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("connection failed (0x%02x)", err);
		return;
	}

	current_conn = bt_conn_ref(conn);
	LOG_INF("central connected");

	/*
	 * Ask for 2M PHY. Doubles the on-air rate, which is what makes
	 * 8ch @ 1 kSPS fit with headroom.
	 */
	const struct bt_conn_le_phy_param phy = {
		.options = BT_CONN_LE_PHY_OPT_NONE,
		.pref_tx_phy = BT_GAP_LE_PHY_2M,
		.pref_rx_phy = BT_GAP_LE_PHY_2M,
	};
	(void)bt_conn_le_phy_update(conn, &phy);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	LOG_INF("central disconnected (reason 0x%02x)", reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	stream_subscribed = false;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);
	/* Interval is in 1.25 ms units; report it in microseconds. */
	LOG_INF("conn params: interval %u us, latency %u, timeout %u ms",
		interval * 1250U, latency, timeout * 10U);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
};

/* --- Advertising ------------------------------------------------------- */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Service UUID in the scan response: it does not fit alongside the name. */
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, SWIFTEEG_UUID_SERVICE_VAL),
};

int ble_transport_init(void)
{
	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return err;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("advertising failed to start (%d)", err);
		return err;
	}

	LOG_INF("BLE advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
	return 0;
}

bool ble_transport_is_streaming(void)
{
	return current_conn != NULL && stream_subscribed;
}

int ble_transport_send_stream(const void *data, uint16_t len)
{
	if (!ble_transport_is_streaming()) {
		return -ENOTCONN;
	}

	/* Attribute 2 is the stream characteristic's value. */
	return bt_gatt_notify(current_conn, &swifteeg_svc.attrs[3], data, len);
}

#else /* !CONFIG_BT */

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble, CONFIG_LOG_DEFAULT_LEVEL);

int ble_transport_init(void)
{
	LOG_INF("BLE disabled in this build");
	return 0;
}

bool ble_transport_is_streaming(void)
{
	return false;
}

int ble_transport_send_stream(const void *data, uint16_t len)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	return -ENOTSUP;
}

#endif /* CONFIG_BT */
