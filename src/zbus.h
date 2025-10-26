#ifndef _ZBUS_H
#define _ZBUS_H

#include <stdint.h>

#include <zephyr/dsp/types.h>
#include <zephyr/zbus/zbus.h>

ZBUS_CHAN_DECLARE(env_chan);
ZBUS_CHAN_DECLARE(timer_chan);

struct env_value {
	q31_t value;
	int8_t shift;
};

struct env_msg {
	struct env_value temp;
	struct env_value humid;
	struct env_value press;
};

#endif /* _ZBUS_H */
