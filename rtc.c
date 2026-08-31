// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller RTC Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

#include "ssaiot_sc.h"

/* Response payload of CMD_SENSOR_READ / SENSOR_RTC */
struct ssaiot_sc_rtc_time {
	u8 year;	/* 0..99, epoch 2000 */
	u8 month;	/* 1..12 */
	u8 day;		/* 1..31 */
	u8 hour;
	u8 min;
	u8 sec;
} __packed;

/**
 * ssaiot_sc_rtc_read_time() - Read the controller's calendar
 * @dev: The RTC sub-device
 * @tm: Where to store the time, always UTC
 *
 * The controller keeps its own calendar and sets it without involvement from
 * the host, so the only failure that matters here is that it has not managed
 * to do so yet.
 */
static int ssaiot_sc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct ssaiot_sc_priv *sc = dev_get_drvdata(dev->parent);
	struct ssaiot_sc_rtc_time resp;
	u8 status;
	int ret;

	ret = ssaiot_sc_xfer(sc, SSAIOT_SC_CMD_SENSOR_READ, SSAIOT_SC_SENSOR_RTC,
			     NULL, 0, (u8 *)&resp, sizeof(resp), &status);
	if (ret)
		return ret;

	/*
	 * The calendar has not been set yet. The six bytes are still returned
	 * but they are the power-on default and mean nothing, so report the
	 * time as unusable rather than hand back a plausible wrong one. The
	 * controller may take a long time to acquire a time, and may never
	 * manage it, so this is a normal state to read back rather than an
	 * error to recover from - a caller that wants the time simply reads
	 * again later.
	 */
	if (status != SSAIOT_SC_STATUS_OK)
		return -EINVAL;

	tm->tm_year = resp.year + 100;
	tm->tm_mon = resp.month - 1;
	tm->tm_mday = resp.day;
	tm->tm_hour = resp.hour;
	tm->tm_min = resp.min;
	tm->tm_sec = resp.sec;

	return 0;
}

/*
 * Read only: the host has no path into the controller's calendar, so there is
 * no set_time. Without set_alarm the RTC core clears RTC_FEATURE_ALARM for us,
 * so no alarm interface is offered either.
 */
static const struct rtc_class_ops ssaiot_sc_rtc_ops = {
	.read_time = ssaiot_sc_rtc_read_time,
};

static int ssaiot_sc_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtc_device *rtc;

	/* the mfd cell has no dedicated dt node, reuse parent */
	dev->of_node = dev->parent->of_node;

	rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	rtc->ops = &ssaiot_sc_rtc_ops;

	/* the calendar carries a two digit year against a 2000 epoch */
	rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	rtc->range_max = RTC_TIMESTAMP_END_2099;

	return devm_rtc_register_device(rtc);
}

/*
 * The id must match the MFD cell name and is capped at PLATFORM_NAME_SIZE,
 * so it stays short. Since an id table suppresses the driver name fallback in
 * platform_match(), the driver name itself is free to be descriptive.
 */
static const struct platform_device_id ssaiot_sc_rtc_id_table[] = {
	{ "ssaiot-sc-rtc", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, ssaiot_sc_rtc_id_table);

static struct platform_driver ssaiot_sc_rtc_driver = {
	.driver = {
		.name = "solidsense-aiot-system-controller-rtc",
	},
	.probe = ssaiot_sc_rtc_probe,
	.id_table = ssaiot_sc_rtc_id_table,
};
module_platform_driver(ssaiot_sc_rtc_driver);

MODULE_AUTHOR("Josua Mayer");
MODULE_DESCRIPTION("SolidRun SolidSense AIOT Board System Controller RTC Driver");
MODULE_LICENSE("GPL v2");
