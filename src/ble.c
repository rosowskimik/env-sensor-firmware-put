#include "zbus.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/dsp/print_format.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <zephyr/zbus/zbus.h>

#include <sys/cdefs.h>
#include <sys/errno.h>
#include <sys/types.h>

LOG_MODULE_REGISTER(ble, CONFIG_APP_LOG_LEVEL);

#define SENSOR_TEMP_NAME  "Temperature Sensor"
#define SENSOR_PRESS_NAME "Pressure Sensor"
#define SENSOR_HUMID_NAME "Humidity Sensor"

/* ESS Measurement Descriptor – Sampling Function */
enum bt_es_measurement_sampling_function {
	ES_MEASUREMENT_SAMPLING_UNSPECIFIED = 0x00,
	ES_MEASUREMENT_SAMPLING_INSTANTANEOUS = 0x01,
	ES_MEASUREMENT_SAMPLING_ARITHMETIC_MEAN = 0x02,
	ES_MEASUREMENT_SAMPLING_RMS = 0x03,
	ES_MEASUREMENT_SAMPLING_MAXIMUM = 0x04,
	ES_MEASUREMENT_SAMPLING_MINIMUM = 0x05,
	ES_MEASUREMENT_SAMPLING_ACCUMULATED = 0x06,
	ES_MEASUREMENT_SAMPLING_COUNT = 0x07,
};

/* ES Measurement Descriptor - Measurement Period */
enum bt_es_measurement_period {
	ES_MEASUREMENT_PERIOD_UNSPECIFIED = 0x00,
};

/* ES Measurement Descriptor - Application */
enum bt_es_measurement_application {
	ES_MEASUREMENT_APP_UNSPECIFIED = 0x00,
	ES_MEASUREMENT_APP_AIR = 0x01,
	ES_MEASUREMENT_APP_WATER = 0x02,
	ES_MEASUREMENT_APP_BAROMETRIC = 0x03,
	ES_MEASUREMENT_APP_SOIL = 0x04,
	ES_MEASUREMENT_APP_INFRARED = 0x05,
	ES_MEASUREMENT_APP_MAP_DATABASE = 0x06,
	ES_MEASUREMENT_APP_BAROMETRIC_ELEVATION_SOURCE = 0x07,
	ES_MEASUREMENT_APP_GPS_ONLY_ELEVATION_SOURCE = 0x08,
	ES_MEASUREMENT_APP_GPS_AND_MAP_DATABASE_ELEVATION_SOURCE = 0x09,
	ES_MEASUREMENT_APP_VERTICAL_DATUM_ELEVATION_SOURCE = 0x0A,
	ES_MEASUREMENT_APP_ONSHORE = 0x0B,
	ES_MEASUREMENT_APP_ONBOARD_VESSEL_OR_VEHICLE = 0x0C,
	ES_MEASUREMENT_APP_FRONT = 0x0D,
	ES_MEASUREMENT_APP_BACK_REAR = 0x0E,
	ES_MEASUREMENT_APP_UPPER = 0x0F,
	ES_MEASUREMENT_APP_LOWER = 0x10,
	ES_MEASUREMENT_APP_PRIMARY = 0x11,
	ES_MEASUREMENT_APP_SECONDARY = 0x12,
	ES_MEASUREMENT_APP_OUTDOOR = 0x13,
	ES_MEASUREMENT_APP_INDOOR = 0x14,
	ES_MEASUREMENT_APP_TOP = 0x15,
	ES_MEASUREMENT_APP_BOTTOM = 0x16,
	ES_MEASUREMENT_APP_MAIN = 0x17,
	ES_MEASUREMENT_APP_BACKUP = 0x18,
	ES_MEASUREMENT_APP_AUXILIARY = 0x19,
	ES_MEASUREMENT_APP_SUPPLEMENTARY = 0x1A,
	ES_MEASUREMENT_APP_INSIDE = 0x1B,
	ES_MEASUREMENT_APP_OUTSIDE = 0x1C,
	ES_MEASUREMENT_APP_LEFT = 0x1D,
	ES_MEASUREMENT_APP_RIGHT = 0x1E,
	ES_MEASUREMENT_APP_INTERNAL = 0x1F,
	ES_MEASUREMENT_APP_EXTERNAL = 0x20,
	ES_MEASUREMENT_APP_SOLAR = 0x21,
};

/* ES Measurement Descriptor - Measurement Uncertainty */
enum bt_es_measurement_uncertainty {
	ES_MEASUREMENT_UNCERTAINTY_UNKNOWN = 0xFF,
};

/* ES Trigger Setting Descriptor - Trigger Condition */
enum bt_es_trigger_setting_condition {
	ES_TRIGGER_INACTIVE = 0x00,
	ES_TRIGGER_INTERVAL_FIXED = 0x01,
	ES_TRIGGER_INTERVAL_MIN = 0x02,
	ES_TRIGGER_VALUE_CHANGE = 0x03,
	ES_TRIGGER_VALUE_LT_SPEC = 0x04,
	ES_TRIGGER_VALUE_LTE_SPEC = 0x05,
	ES_TRIGGER_VALUE_GT_SPEC = 0x06,
	ES_TRIGGER_VALUE_GTE_SPEC = 0x07,
	ES_TRIGGER_VALUE_EQ_SPEC = 0x08,
	ES_TRIGGER_VALUE_NEQ_SPEC = 0x09,
};

/* https://www.bluetooth.com/specifications/specs/html/?src=ESS_v1.0.1/out/en/index-en.html#UUID-2407bbfc-b4c8-2b7c-7bda-feb4d3cbcfb5_Table_3.3
 */
struct bt_es_measurement_desc {
	uint16_t flags;
	uint8_t sampling_func;
	// uint24
	uint8_t meas_period[3];
	// uint24
	uint8_t update_interval[3];
	uint8_t application;
	uint8_t meas_uncertainty;
} __packed;

/* https://www.bluetooth.com/specifications/specs/html/?src=ESS_v1.0.1/out/en/index-en.html#UUID-0ad8b9f9-19ad-daf7-c103-6addaa3d4154_table-idm53478966500680
 */
struct bt_es_trigger_setting_desc {
	uint8_t cond;
	union {
		struct {
		} inactive;
		uint8_t fixed[3];
		uint8_t min[3];
		struct {
		} change;
	} operand;
};

/* https://btprodspecificationrefs.blob.core.windows.net/gatt-specification-supplement/GATT_Specification_Supplement.pdf
 */
struct bt_es_sensor_data_char {
	// degrees Celsius with resolution of 0.01
	int16_t temp;
	// pascals with resolution of 0.1
	uint32_t press;
	// percent with resolution of 0.01
	uint16_t humid;
};

// https://btprodspecificationrefs.blob.core.windows.net/gatt-specification-supplement/GATT_Specification_Supplement.pdf#4.1.3
// -40 - +85 C
const static int16_t bt_es_sensor_temp_range[2] = {sys_cpu_to_le16(-4000), sys_cpu_to_le16(8500)};

// 300 - 1100 hPa
const static uint32_t bt_es_sensor_press_range[2] = {sys_cpu_to_le32(300000),
						     sys_cpu_to_le32(1100000)};

const static struct bt_es_measurement_desc sens_desc = {
	.flags = sys_cpu_to_le16(0x0),
	.sampling_func = ES_MEASUREMENT_SAMPLING_INSTANTANEOUS,
	.meas_period = {sys_cpu_to_le24(ES_MEASUREMENT_PERIOD_UNSPECIFIED)},
	.update_interval = {sys_cpu_to_le24(CONFIG_APP_SENSOR_INTERVAL)},
	.application = ES_MEASUREMENT_APP_INDOOR,
	.meas_uncertainty = ES_MEASUREMENT_UNCERTAINTY_UNKNOWN,
};

const static struct bt_es_trigger_setting_desc sens_trig = {
	.cond = ES_TRIGGER_INTERVAL_FIXED,
	.operand.fixed = {sys_cpu_to_le24(CONFIG_APP_SENSOR_INTERVAL)}};

static K_MUTEX_DEFINE(env_data_mtx);
static struct bt_es_sensor_data_char env_data;

static ssize_t read_es_measurement(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				   uint16_t len, uint16_t offset)
{
	LOG_DBG("read ES Measurement");
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &sens_desc, sizeof(sens_desc));
}

static size_t bt_es_trigger_setting_size(const struct bt_es_trigger_setting_desc *desc)
{
	size_t size = sizeof(desc->cond);
	switch (desc->cond) {
	case ES_TRIGGER_INACTIVE:
		__fallthrough;
	case ES_TRIGGER_VALUE_CHANGE:
		return size;
	case ES_TRIGGER_INTERVAL_FIXED:
		__fallthrough;
	case ES_TRIGGER_INTERVAL_MIN:
		return size + sizeof(desc->operand.min);
	default:
		return -ENOSYS;
	}
}

static ssize_t read_es_trigger_setting(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				       void *buf, uint16_t len, uint16_t offset)
{
	LOG_DBG("read ES Trigger Setting");
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &sens_trig,
				 bt_es_trigger_setting_size(&sens_trig));
}

static ssize_t read_valid_range_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				     void *buf, uint16_t len, uint16_t offset)
{
	LOG_DBG("read temperature Valid Range");

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &bt_es_sensor_temp_range,
				 sizeof(bt_es_sensor_temp_range));
}

static ssize_t read_valid_range_press(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				      void *buf, uint16_t len, uint16_t offset)
{
	LOG_DBG("read pressure Valid Range");

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &bt_es_sensor_press_range,
				 sizeof(bt_es_sensor_press_range));
}

static ssize_t read_value_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	ssize_t ret;

	LOG_DBG("read temperature value");
	k_mutex_lock(&env_data_mtx, K_FOREVER);

	ret = bt_gatt_attr_read(conn, attr, buf, len, offset, &env_data.temp,
				sizeof(env_data.temp));

	k_mutex_unlock(&env_data_mtx);

	return ret;
}

static ssize_t read_value_press(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				uint16_t len, uint16_t offset)
{
	ssize_t ret;

	LOG_DBG("read pressure value");
	k_mutex_lock(&env_data_mtx, K_FOREVER);

	ret = bt_gatt_attr_read(conn, attr, buf, len, offset, &env_data.press,
				sizeof(env_data.press));

	k_mutex_unlock(&env_data_mtx);

	return ret;
}

static ssize_t read_value_humid(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				uint16_t len, uint16_t offset)
{
	ssize_t ret;

	LOG_DBG("read humidity value");
	k_mutex_lock(&env_data_mtx, K_FOREVER);

	ret = bt_gatt_attr_read(conn, attr, buf, len, offset, &env_data.humid,
				sizeof(env_data.humid));

	k_mutex_unlock(&env_data_mtx);

	return ret;
}

// clang-format off
BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),

	// Temperature
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
						   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
						   BT_GATT_PERM_READ,
						   read_value_temp, NULL, NULL),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
					   read_es_measurement, NULL, NULL),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING, BT_GATT_PERM_READ,
					   read_es_trigger_setting, NULL, NULL),
	BT_GATT_CUD(SENSOR_TEMP_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_VALID_RANGE, BT_GATT_PERM_READ,
					   read_valid_range_temp, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	// Pressure
	BT_GATT_CHARACTERISTIC(BT_UUID_PRESSURE,
						   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
						   BT_GATT_PERM_READ,
						   read_value_press, NULL, NULL),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
					   read_es_measurement, NULL, NULL),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING, BT_GATT_PERM_READ,
					   read_es_trigger_setting, NULL, NULL),
	BT_GATT_CUD(SENSOR_PRESS_NAME, BT_GATT_PERM_READ),
	BT_GATT_DESCRIPTOR(BT_UUID_VALID_RANGE, BT_GATT_PERM_READ,
					   read_valid_range_press, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	// Humididy
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
						   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
						   BT_GATT_PERM_READ,
						   read_value_humid, NULL, NULL),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_MEASUREMENT, BT_GATT_PERM_READ,
					   read_es_measurement, NULL, NULL),
	BT_GATT_DESCRIPTOR(BT_UUID_ES_TRIGGER_SETTING, BT_GATT_PERM_READ,
					   read_es_trigger_setting, NULL, NULL),
	BT_GATT_CUD(SENSOR_HUMID_NAME, BT_GATT_PERM_READ),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
				  BT_BYTES_LIST_LE16(CONFIG_BT_DEVICE_APPEARANCE)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
				  BT_UUID_16_ENCODE(BT_UUID_ESS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE,
			CONFIG_BT_DEVICE_NAME,
			sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
// clang-format on

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("connection failed (error: %d - %s)", err, bt_hci_err_to_str(err));
	} else {
		LOG_INF("connected");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("disconnected - %s", bt_hci_err_to_str(reason));
}

static void recycled(void)
{
	int rc;
	LOG_DBG("connection recycled");

	rc = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (rc) {
		LOG_ERR("advertising failed to start (error: %d)", rc);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
};

static void env_data_update(const struct env_msg *msg)
{
	char buf[8];
	k_mutex_lock(&env_data_mtx, K_FOREVER);

	snprintf(buf, ARRAY_SIZE(buf), "%s%d%02d", PRIq_arg(msg->temp.value, 2, msg->temp.shift));
	env_data.temp = sys_cpu_to_le16(strtol(buf, NULL, 10));

	snprintf(buf, ARRAY_SIZE(buf), "%s%d%04d", PRIq_arg(msg->press.value, 4, msg->press.shift));
	env_data.press = sys_cpu_to_le32(strtoul(buf, NULL, 10));

	snprintf(buf, ARRAY_SIZE(buf), "%s%d%02d", PRIq_arg(msg->humid.value, 2, msg->humid.shift));
	env_data.humid = sys_cpu_to_le16(strtoul(buf, NULL, 10));

	k_mutex_unlock(&env_data_mtx);

	bt_gatt_notify(NULL, &ess_svc.attrs[2], &env_data.temp, sizeof(env_data.temp));
	bt_gatt_notify(NULL, &ess_svc.attrs[9], &env_data.press, sizeof(env_data.press));
	bt_gatt_notify(NULL, &ess_svc.attrs[16], &env_data.humid, sizeof(env_data.humid));
}

ZBUS_MSG_SUBSCRIBER_DEFINE(bluetooth_msg_subscriber);
ZBUS_CHAN_ADD_OBS(env_chan, bluetooth_msg_subscriber, 1);

static void bluetooth_thrd()
{
	const struct zbus_channel *chan;
	struct env_msg msg = {0};
	int rc;

	if ((rc = bt_enable(NULL))) {
		LOG_ERR("initialization failed (error: %d)", rc);
		return;
	}

	LOG_INF("initialized");

	rc = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (rc) {
		LOG_ERR("advertising failed to start (error: %d)", rc);
		return;
	}

	LOG_INF("advertising started");

	for (;;) {
		if ((rc = zbus_sub_wait_msg(&bluetooth_msg_subscriber, &chan, &msg, K_FOREVER))) {
			LOG_ERR("waiting for channel message failed (error: %d)", rc);
			continue;
		}

		env_data_update(&msg);
	}
}

K_THREAD_DEFINE(bluetooth_thrd_id, CONFIG_APP_BLE_STACK_SIZE, bluetooth_thrd, NULL, NULL, NULL,
		CONFIG_APP_BLE_THREAD_PRIORITY, 0, 0);
