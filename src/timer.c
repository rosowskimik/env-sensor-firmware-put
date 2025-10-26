#include "zbus.h"

#include <zephyr/kernel.h>
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

static void timer_init_thrd()
{
	k_timer_start(&env_timer, K_SECONDS(1), K_SECONDS(CONFIG_APP_SENSOR_INTERVAL));
	LOG_INF("timer started");
}

K_THREAD_DEFINE(timer_thrd_id, CONFIG_APP_TIMER_STACK_SIZE, timer_init_thrd, NULL, NULL, NULL,
		CONFIG_APP_TIMER_THREAD_PRIORITY, 0, 0);
