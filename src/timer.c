#include "zbus.h"

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

LOG_MODULE_REGISTER(timer, CONFIG_APP_LOG_LEVEL);

/* clang-format off */
ZBUS_CHAN_DEFINE(timer_chan,	/* Name */
	 void*,						/* Message type */
	 NULL,						/* Validator */
	 NULL,						/* User data */
	 ZBUS_OBSERVERS_EMPTY,		/* Observers */
	 ZBUS_MSG_INIT(NULL)		/* Initial value */
);
/* clang-format on */

static void timer_expiry(struct k_timer *timer)
{
	int rc;

	LOG_DBG("timer expired");

	if ((rc = zbus_chan_pub(&timer_chan, timer, K_NO_WAIT))) {
		LOG_ERR("failed to publish timer event to zbus (error: %d)", rc);
	}
}

K_TIMER_DEFINE(env_timer, timer_expiry, NULL);

static int timer_init()
{
	k_timer_start(&env_timer, K_SECONDS(3), K_SECONDS(CONFIG_APP_SENSOR_INTERVAL));
	LOG_INF("timer started");
	return 0;
}

SYS_INIT(timer_init, APPLICATION, CONFIG_APP_TIMER_INIT_PRIORITY);
