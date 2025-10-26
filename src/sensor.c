#include "zbus.h"

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/dsp/print_format.h>
#include <zephyr/dsp/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/util.h>
#include <zephyr/zbus/zbus.h>

LOG_MODULE_REGISTER(sensor, CONFIG_APP_LOG_LEVEL);

#define SENSOR_DEV DT_ALIAS(env_sensor)

const static struct device *const dev = DEVICE_DT_GET(SENSOR_DEV);

#define SENS_CHAN(spec) ((struct sensor_chan_spec){spec})

#define SENS_CHAN_TEMP  SENS_CHAN(SENSOR_CHAN_AMBIENT_TEMP)
#define SENS_CHAN_HUMID SENS_CHAN(SENSOR_CHAN_HUMIDITY)
#define SENS_CHAN_PRESS SENS_CHAN(SENSOR_CHAN_PRESS)

/* clang-format off */
SENSOR_DT_READ_IODEV(iodev, SENSOR_DEV,
	SENS_CHAN_TEMP,
	SENS_CHAN_HUMID,
	SENS_CHAN_PRESS,
);
/* clang-format on */

RTIO_DEFINE(ctx, 1, 1);

ZBUS_SUBSCRIBER_DEFINE(env_subscriber, 1);
ZBUS_CHAN_ADD_OBS(timer_chan, env_subscriber, 0);

/* clang-format off */
ZBUS_CHAN_DEFINE(env_chan,	/* Name */
	 struct env_msg,		/* Message */
	 NULL,					/* Validator */
	 NULL,					/* User data */
	 ZBUS_OBSERVERS_EMPTY,	/* Observers */
	 ZBUS_MSG_INIT(0)		/* Initial value */
);
/* clang-format on */

static void sensor_thrd(void)
{
	const struct sensor_decoder_api *decoder;
	const struct zbus_channel *chan;
	struct sensor_q31_data data;
	struct env_msg msg;
	uint8_t buf[128];
	int rc, fit;

	if (!device_is_ready(dev)) {
		LOG_ERR("%s: device not ready", dev->name);
		return;
	}

	if ((rc = sensor_get_decoder(dev, &decoder))) {
		LOG_ERR("%s: failed to get decoder (error: %d)", dev->name, rc);
		return;
	}

	LOG_INF("%s: thread started", dev->name);

	for (;;) {
		if ((rc = zbus_sub_wait(&env_subscriber, &chan, K_FOREVER))) {
			LOG_ERR("%s: waiting for channel notification failed (error: %d)",
				dev->name, rc);
			continue;
		}

		LOG_DBG("%s: reading sensor", dev->name);
		if ((rc = sensor_read(&iodev, &ctx, buf, ARRAY_SIZE(buf)))) {
			LOG_ERR("%s: failed to read sensor data (error: %d)", dev->name, rc);
			continue;
		}

		fit = 0;
		decoder->decode(buf, SENS_CHAN_TEMP, &fit, 1, &data);
		msg.temp = (struct env_value){data.readings[0].temperature, data.shift};

		fit = 0;
		decoder->decode(buf, SENS_CHAN_HUMID, &fit, 1, &data);
		msg.humid = (struct env_value){data.readings[0].humidity, data.shift};

		fit = 0;
		decoder->decode(buf, SENS_CHAN_PRESS, &fit, 1, &data);
		msg.press = (struct env_value){data.readings[0].pressure, data.shift};

		LOG_DBG("%s: sensor reading: temp - %s%d.%04d°C, humid - %s%d.%02d%%, press - "
			"%s%d.%04dkPa",
			dev->name, PRIq_arg(msg.temp.value, 4, msg.temp.shift),
			PRIq_arg(msg.humid.value, 2, msg.humid.shift),
			PRIq_arg(msg.press.value, 4, msg.press.shift));

		if ((rc = zbus_chan_pub(&env_chan, &msg, K_NO_WAIT))) {
			LOG_ERR("%s: failed to publish sensor data to zbus (error: %d)", dev->name,
				rc);
			continue;
		}
	}
}

K_THREAD_DEFINE(sensor_thrd_id, CONFIG_APP_SENSOR_STACK_SIZE, sensor_thrd, NULL, NULL, NULL,
		CONFIG_APP_SENSOR_THREAD_PRIORITY, 0, 0);
