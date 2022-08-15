/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Bitbang driver for Linux GPIO descriptors through libgpiod
 * Copyright (C) 2020 Antonio Borneo <borneo.antonio@gmail.com>
 *
 * Largely based on sysfsgpio driver
 * Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au
 * Copyright (C) 2014 by Jean-Christian de Rivaz <jc@eclis.ch>
 * Copyright (C) 2014 by Paul Fertser <fercerpav@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gpiod.h>
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

#define LOG_PREFIX "linuxgpiod: "

static const char *consumer = "OpenOCD";
static struct gpiod_chip *gpiod_chip[ADAPTER_GPIO_IDX_NUM] = {};
static struct gpiod_line *gpiod_line[ADAPTER_GPIO_IDX_NUM] = {};

static int last_swclk;
static int last_swdio;
static bool last_stored;
static bool swdio_input;

static const struct adapter_gpio_config *adapter_gpio_config;

/*
 * Helper function to determine if gpio config is valid
 *
 * Assume here that there will be less than 10000 gpios per gpiochip, and less
 * than 1000 gpiochips.
 */
static bool is_gpio_config_valid(enum adapter_gpio_config_index idx)
{
	return adapter_gpio_config[idx].chip_num >= 0
		&& adapter_gpio_config[idx].chip_num < 1000
		&& adapter_gpio_config[idx].gpio_num >= 0
		&& adapter_gpio_config[idx].gpio_num < 10000;
}

/* Bitbang interface read of TDO */
static bb_value_t linuxgpiod_read(void)
{
	int retval;

	retval = gpiod_line_get_value(gpiod_line[ADAPTER_GPIO_IDX_TDO]);
	if (retval < 0) {
		LOG_WARNING(LOG_PREFIX "reading tdo failed");
		return 0;
	}

	return retval ? BB_HIGH : BB_LOW;
}

/*
 * Bitbang interface write of TCK, TMS, TDI
 *
 * Seeing as this is the only function where the outputs are changed,
 * we can cache the old value to avoid needlessly writing it.
 */
static int linuxgpiod_write(int tck, int tms, int tdi)
{
	static int last_tck;
	static int last_tms;
	static int last_tdi;

	static int first_time;

	int retval;

	if (!first_time) {
		last_tck = !tck;
		last_tms = !tms;
		last_tdi = !tdi;
		first_time = 1;
	}

	if (tdi != last_tdi) {
		retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_TDI], tdi);
		if (retval < 0)
			LOG_WARNING(LOG_PREFIX "writing tdi failed");
	}

	if (tms != last_tms) {
		retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_TMS], tms);
		if (retval < 0)
			LOG_WARNING(LOG_PREFIX "writing tms failed");
	}

	/* write clk last */
	if (tck != last_tck) {
		retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_TCK], tck);
		if (retval < 0)
			LOG_WARNING(LOG_PREFIX "writing tck failed");
	}

	last_tdi = tdi;
	last_tms = tms;
	last_tck = tck;

	return ERROR_OK;
}

static int linuxgpiod_swdio_read(void)
{
	int retval;

	retval = gpiod_line_get_value(gpiod_line[ADAPTER_GPIO_IDX_SWDIO]);
	if (retval < 0) {
		LOG_WARNING(LOG_PREFIX "Fail read swdio");
		return 0;
	}

	return retval;
}

static void linuxgpiod_swdio_drive(bool is_output)
{
	int retval;

	/*
	 * FIXME: change direction requires release and re-require the line
	 * https://stackoverflow.com/questions/58735140/
	 * this would change in future libgpiod
	 */
	gpiod_line_release(gpiod_line[ADAPTER_GPIO_IDX_SWDIO]);

	if (is_output) {
		if (gpiod_line[ADAPTER_GPIO_IDX_SWDIO_DIR]) {
			retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SWDIO_DIR], 1);
			if (retval < 0)
				LOG_WARNING(LOG_PREFIX "Fail set swdio_dir");
		}
		retval = gpiod_line_request_output(gpiod_line[ADAPTER_GPIO_IDX_SWDIO], consumer, 1);
		if (retval < 0)
			LOG_WARNING(LOG_PREFIX "Fail request_output line swdio");
	} else {
		retval = gpiod_line_request_input(gpiod_line[ADAPTER_GPIO_IDX_SWDIO], consumer);
		if (retval < 0)
			LOG_WARNING(LOG_PREFIX "Fail request_input line swdio");
		if (gpiod_line[ADAPTER_GPIO_IDX_SWDIO_DIR]) {
			retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SWDIO_DIR], 0);
			if (retval < 0)
				LOG_WARNING(LOG_PREFIX "Fail set swdio_dir");
		}
	}

	last_stored = false;
	swdio_input = !is_output;
}

static int linuxgpiod_swd_write(int swclk, int swdio)
{
	int retval;

	if (!swdio_input) {
		if (!last_stored || (swdio != last_swdio)) {
			retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SWDIO], swdio);
			if (retval < 0)
				LOG_WARNING(LOG_PREFIX "Fail set swdio");
		}
	}

	/* write swclk last */
	if (!last_stored || (swclk != last_swclk)) {
		retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SWCLK], swclk);
		if (retval < 0)
			LOG_WARNING(LOG_PREFIX "Fail set swclk");
	}

	last_swdio = swdio;
	last_swclk = swclk;
	last_stored = true;

	return ERROR_OK;
}

static int linuxgpiod_blink(int on)
{
	int retval;

	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_LED))
		return ERROR_OK;

	retval = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_LED], on);
	if (retval < 0)
		LOG_WARNING(LOG_PREFIX "Fail set led");
	return retval;
}

static struct bitbang_interface linuxgpiod_bitbang = {
	.read = linuxgpiod_read,
	.write = linuxgpiod_write,
	.swdio_read = linuxgpiod_swdio_read,
	.swdio_drive = linuxgpiod_swdio_drive,
	.swd_write = linuxgpiod_swd_write,
	.blink = linuxgpiod_blink,
};

/*
 * Bitbang interface to manipulate reset lines SRST and TRST
 *
 * (1) assert or (0) deassert reset lines
 */
static int linuxgpiod_reset(int trst, int srst)
{
	int retval1 = 0, retval2 = 0;

	LOG_DEBUG(LOG_PREFIX "linuxgpiod_reset");

	/*
	 * active low behaviour handled by "adaptor gpio" command and
	 * GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW flag when requesting the line.
	 */
	if (gpiod_line[ADAPTER_GPIO_IDX_SRST]) {
		retval1 = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_SRST], srst);
		if (retval1 < 0)
			LOG_WARNING(LOG_PREFIX "set srst value failed");
	}

	if (gpiod_line[ADAPTER_GPIO_IDX_TRST]) {
		retval2 = gpiod_line_set_value(gpiod_line[ADAPTER_GPIO_IDX_TRST], trst);
		if (retval2 < 0)
			LOG_WARNING(LOG_PREFIX "set trst value failed");
	}

	return ((retval1 < 0) || (retval2 < 0)) ? ERROR_FAIL : ERROR_OK;
}

static bool linuxgpiod_jtag_mode_possible(void)
{
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TCK))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TMS))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDI))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDO))
		return false;
	return true;
}

static bool linuxgpiod_swd_mode_possible(void)
{
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_SWCLK))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_SWDIO))
		return false;
	return true;
}

static inline void helper_release(enum adapter_gpio_config_index idx)
{
	if (gpiod_line[idx]) {
		gpiod_line_release(gpiod_line[idx]);
		gpiod_line[idx] = NULL;
	}
	if (gpiod_chip[idx]) {
		gpiod_chip_close(gpiod_chip[idx]);
		gpiod_chip[idx] = NULL;
	}
}

static int linuxgpiod_quit(void)
{
	LOG_DEBUG(LOG_PREFIX "linuxgpiod_quit");
	for (int i = 0; i < ADAPTER_GPIO_IDX_NUM; ++i)
		helper_release(i);

	return ERROR_OK;
}

int helper_get_line(enum adapter_gpio_config_index idx)
{
	if (!is_gpio_config_valid(idx))
		return ERROR_OK;

	int request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT, flags = 0, val = 0, retval;

	gpiod_chip[idx] = gpiod_chip_open_by_number(adapter_gpio_config[idx].chip_num);
	if (!gpiod_chip[idx]) {
		LOG_ERROR(LOG_PREFIX "Cannot open LinuxGPIOD chip %d for %s", adapter_gpio_config[idx].chip_num,
			adapter_gpio_get_name(idx));
		return ERROR_JTAG_INIT_FAILED;
	}

	gpiod_line[idx] = gpiod_chip_get_line(gpiod_chip[idx], adapter_gpio_config[idx].gpio_num);
	if (!gpiod_line[idx]) {
		LOG_ERROR(LOG_PREFIX "Error get line %s", adapter_gpio_get_name(idx));
		return ERROR_JTAG_INIT_FAILED;
	}

	switch (adapter_gpio_config[idx].init_state) {
	case ADAPTER_GPIO_INIT_STATE_INPUT:
		request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
		break;
	case ADAPTER_GPIO_INIT_STATE_INACTIVE:
		request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
		val = 0;
		break;
	case ADAPTER_GPIO_INIT_STATE_ACTIVE:
		request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
		val = 1;
		break;
	}
	if (idx == ADAPTER_GPIO_IDX_PWR_SENSE || idx == ADAPTER_GPIO_IDX_SRST_SENSE)
		request_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES;

	switch (adapter_gpio_config[idx].drive) {
	case ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL:
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN:
		flags |= GPIOD_LINE_REQUEST_FLAG_OPEN_DRAIN;
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE:
		flags |= GPIOD_LINE_REQUEST_FLAG_OPEN_SOURCE;
		break;
	}

	switch (adapter_gpio_config[idx].pull) {
	case ADAPTER_GPIO_PULL_NONE:
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE
		flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE;
#endif
		break;
	case ADAPTER_GPIO_PULL_UP:
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP
		flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
#else
		LOG_WARNING(LOG_PREFIX "ignoring request for pull-up on %s: not supported by gpiod v%s",
			adapter_gpio_get_name(idx), gpiod_version_string());
#endif
		break;
	case ADAPTER_GPIO_PULL_DOWN:
#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN
		flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
#else
		LOG_WARNING(LOG_PREFIX "ignoring request for pull-down on %s: not supported by gpiod v%s",
			adapter_gpio_get_name(idx), gpiod_version_string());
#endif
		break;
	}

	if (adapter_gpio_config[idx].active_low)
		flags |= GPIOD_LINE_REQUEST_FLAG_ACTIVE_LOW;

	struct gpiod_line_request_config config = {
		.consumer = consumer,
		.request_type = request_type,
		.flags = flags,
	};

	retval = gpiod_line_request(gpiod_line[idx], &config, val);
	if (retval < 0) {
		LOG_ERROR(LOG_PREFIX "Error requesting gpio line %s", adapter_gpio_get_name(idx));
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int linuxgpiod_init(void)
{
	LOG_INFO("Linux GPIOD JTAG/SWD bitbang driver");

	bitbang_interface = &linuxgpiod_bitbang;
	adapter_gpio_config = adapter_gpio_get_config();

	/* Apply power first */
	if (helper_get_line(ADAPTER_GPIO_IDX_PWR_CTRL) != ERROR_OK)
		goto out_error;

	/*
	 * Configure JTAG/SWD signals. Default directions and initial states are handled
	 * by adapter.c and "adapter gpio" command.
	 */

	if (transport_is_jtag()) {
		if (!linuxgpiod_jtag_mode_possible()) {
			LOG_ERROR(LOG_PREFIX "Require tck, tms, tdi and tdo gpios for JTAG mode");
			goto out_error;
		}

		if (helper_get_line(ADAPTER_GPIO_IDX_TDO) != ERROR_OK ||
			helper_get_line(ADAPTER_GPIO_IDX_TDI) != ERROR_OK ||
			helper_get_line(ADAPTER_GPIO_IDX_TCK) != ERROR_OK ||
			helper_get_line(ADAPTER_GPIO_IDX_TMS) != ERROR_OK ||
			helper_get_line(ADAPTER_GPIO_IDX_TRST) != ERROR_OK)
				goto out_error;
	}

	if (transport_is_swd()) {
		int retval1, retval2;
		if (!linuxgpiod_swd_mode_possible()) {
			LOG_ERROR(LOG_PREFIX "Require swclk and swdio gpio for SWD mode");
			goto out_error;
		}

		/*
		 * swdio and its buffer should be initialized in the order that prevents
		 * two outputs from being connected together. This will occur if the
		 * swdio GPIO is configured as an output while the external buffer is
		 * configured to send the swdio signal from the target to the GPIO.
		 */
		if (adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].init_state == ADAPTER_GPIO_INIT_STATE_INPUT) {
			retval1 = helper_get_line(ADAPTER_GPIO_IDX_SWDIO);
			retval2 = helper_get_line(ADAPTER_GPIO_IDX_SWDIO_DIR);
		} else {
			retval1 = helper_get_line(ADAPTER_GPIO_IDX_SWDIO_DIR);
			retval2 = helper_get_line(ADAPTER_GPIO_IDX_SWDIO);
		}
		if (retval1 != ERROR_OK || retval2 != ERROR_OK)
			goto out_error;

		if (helper_get_line(ADAPTER_GPIO_IDX_SWCLK) != ERROR_OK)
			goto out_error;
	}

	if (helper_get_line(ADAPTER_GPIO_IDX_SRST) != ERROR_OK ||
		helper_get_line(ADAPTER_GPIO_IDX_LED) != ERROR_OK ||
		helper_get_line(ADAPTER_GPIO_IDX_SRST_SENSE) != ERROR_OK ||
		helper_get_line(ADAPTER_GPIO_IDX_PWR_SENSE) != ERROR_OK)
			goto out_error;

	return ERROR_OK;

out_error:
	linuxgpiod_quit();

	return ERROR_JTAG_INIT_FAILED;
}

/**
 * @brief Sense changes on a line
 *
 * @param idx: GPIO to read
 * @param asserted: flag inidcating if line is active, caller should maintain state between calls
 * @param initialized: flag indicating if the asserted parameter has been initialized. Read the line if not.
 * @return int
 */
static int linuxgpiod_line_sense(enum adapter_gpio_config_index idx, int *asserted, bool *initialized)
{
	if (!is_gpio_config_valid(idx)) {
		*asserted = 0;
		return ERROR_OK;
	}

	/* Check if any events occurred since last time; read out all. gpiod inverts
	 * rising/falling edge when signal is active-low. */
	int num_events = 0;
	int num_assert_events = 0;
	int num_deassert_events = 0;

	while (true) {
		const struct timespec timeout = {
			.tv_sec = 0,
			.tv_nsec = 0,
		};
		int r = gpiod_line_event_wait(gpiod_line[idx], &timeout);
		if (r < 0) {
			LOG_ERROR(LOG_PREFIX "gpiod_line_event_wait() failed for signal %s: %s",
				adapter_gpio_get_name(idx), strerror(errno));
			return ERROR_FAIL;
		}
		if (!r)
			break;

		const int max_num_events = 4;
		struct gpiod_line_event events[max_num_events];
		num_events = gpiod_line_event_read_multiple(gpiod_line[idx], events, max_num_events);
		if (num_events < 0) {
			LOG_ERROR(LOG_PREFIX "gpiod_line_event_read_multiple() failed for signal %s: %s",
				adapter_gpio_get_name(idx), strerror(errno));
			return ERROR_FAIL;
		}

		for (int i = 0; i < num_events; ++i) {
			LOG_DEBUG(LOG_PREFIX "event type %d", events[i].event_type);
			switch (events[i].event_type) {
			case GPIOD_LINE_EVENT_RISING_EDGE:
				++num_assert_events;
				break;
			case GPIOD_LINE_EVENT_FALLING_EDGE:
				++num_deassert_events;
				break;
			}
		}
	}

	/* The value of asserted is changed only if events were detected. Make it
	 * the caller's responsibility to store the previous state. Then it is not
	 * necessary to read the GPIO level after initialization. */
	if (num_deassert_events)
		*asserted = false;
	if (num_assert_events || num_deassert_events > 1)
		/* It has been observed that gpiod can return consecutive events of the
		 * same type. If multiple deassert events were detected then there must
		 * have been at least one assert event. */
		*asserted = true;

	if (!initialized) {
		if (!num_assert_events && !num_deassert_events) {
			int retval = gpiod_line_get_value(gpiod_line[idx]);
			if (retval < 0) {
				LOG_ERROR(LOG_PREFIX "reading %s failed: %s", adapter_gpio_get_name(idx), strerror(errno));
				return ERROR_FAIL;
			}
			*asserted = (retval ? 1 : 0);
		}
		*initialized = true;
	}

	return ERROR_OK;
}

static int linuxgpiod_power_dropout(int *power_dropout)
{
	static int active;
	static bool initialized;
	int r = linuxgpiod_line_sense(ADAPTER_GPIO_IDX_PWR_SENSE, &active, &initialized);
	*power_dropout = active;
	return r;
}

static int linuxgpiod_srst_asserted(int *srst_asserted)
{
	static int active;
	static bool initialized;
	int r = linuxgpiod_line_sense(ADAPTER_GPIO_IDX_SRST_SENSE, &active, &initialized);
	*srst_asserted = active;
	return r;
}

static const char *const linuxgpiod_transport[] = { "swd", "jtag", NULL };

static struct jtag_interface linuxgpiod_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver linuxgpiod_adapter_driver = {
	.name = "linuxgpiod",
	.transports = linuxgpiod_transport,

	.init = linuxgpiod_init,
	.quit = linuxgpiod_quit,
	.reset = linuxgpiod_reset,
	.power_dropout = linuxgpiod_power_dropout,
	.srst_asserted = linuxgpiod_srst_asserted,

	.jtag_ops = &linuxgpiod_interface,
	.swd_ops = &bitbang_swd,
};
