// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller RTC Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/interrupt.h>
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

/* Payload of CMD_SENSOR_(CONFIG/READ) / SENSOR_ALARM, binary and 24 hour */
struct ssaiot_sc_rtc_alarm {
	u8 hour;
	u8 min;
	u8 sec;
} __packed;

struct ssaiot_sc_rtc {
	struct ssaiot_sc_priv *sc;
	struct rtc_device *rtc;
	int irq;
};

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
	struct ssaiot_sc_rtc *rtc = dev_get_drvdata(dev);
	struct ssaiot_sc_rtc_time resp;
	u8 status;
	int ret;

	ret = ssaiot_sc_xfer(rtc->sc, SSAIOT_SC_CMD_SENSOR_READ,
			     SSAIOT_SC_SENSOR_RTC, NULL, 0,
			     (u8 *)&resp, sizeof(resp), &status);
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

static int ssaiot_sc_rtc_alarm_cancel(struct ssaiot_sc_rtc *rtc)
{
	u8 status;
	int ret;

	/* cancelling is its own command, not a magic time */
	ret = ssaiot_sc_xfer(rtc->sc, SSAIOT_SC_CMD_SENSOR_OFF,
			     SSAIOT_SC_SENSOR_ALARM, NULL, 0, NULL, 0, &status);
	if (ret)
		return ret;

	return status == SSAIOT_SC_STATUS_OK ? 0 : -EIO;
}

static int ssaiot_sc_rtc_alarm_arm(struct ssaiot_sc_rtc *rtc,
				   const struct ssaiot_sc_rtc_alarm *time)
{
	u8 status;
	int ret;

	ret = ssaiot_sc_xfer(rtc->sc, SSAIOT_SC_CMD_SENSOR_CONFIG,
			     SSAIOT_SC_SENSOR_ALARM, (const u8 *)time,
			     sizeof(*time), NULL, 0, &status);
	if (ret)
		return ret;

	/* the controller range checks the fields and rejects the whole command */
	if (status != SSAIOT_SC_STATUS_OK)
		return -EINVAL;

	return 0;
}

/* read active (armed) alarm, if any */
static int ssaiot_sc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct ssaiot_sc_rtc *rtc = dev_get_drvdata(dev);
	time64_t now_secs, alarm_secs;
	struct ssaiot_sc_rtc_alarm resp;
	struct rtc_time now;
	u8 status;
	int ret;

	ret = ssaiot_sc_xfer(rtc->sc, SSAIOT_SC_CMD_SENSOR_READ,
			     SSAIOT_SC_SENSOR_ALARM, NULL, 0,
			     (u8 *)&resp, sizeof(resp), &status);
	if (ret)
		return ret;

	if (status != SSAIOT_SC_STATUS_OK) {
		/* no active alarm */
		alrm->enabled = 0;
		return 0;
	}

	/* read current date&time */
	ret = ssaiot_sc_rtc_read_time(dev, &now);
	if (ret == -EINVAL) {
		/* Special case time not yet set, meaning the next alarm date is unknown. */
		dev_warn(dev, "failed to read date for active alarm, alarm is not tracked\n");
		alrm->enabled = 0;
		return 0;
	} else if (ret) {
		return ret;
	}
	now_secs = rtc_tm_to_time64(&now);

	/* alarm time is without date, calculate today's occurrence */
	alarm_secs = now_secs;
	alarm_secs -= now_secs % 86400;
	alarm_secs += resp.hour * 3600;
	alarm_secs += resp.min * 60;
	alarm_secs += resp.sec;

	/* if today's alarm has passed, roll tomorrow */
	if (alarm_secs <= now_secs)
		alarm_secs += 86400;

	/* convert to rtc_time */
	rtc_time64_to_tm(alarm_secs, &alrm->time);

	/* alarms are always armed */
	alrm->enabled = 1;

	return 0;
}

static int ssaiot_sc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct ssaiot_sc_rtc *rtc = dev_get_drvdata(dev);
	struct ssaiot_sc_rtc_alarm time;

	time.hour = alrm->time.tm_hour;
	time.min = alrm->time.tm_min;
	time.sec = alrm->time.tm_sec;

	return ssaiot_sc_rtc_alarm_arm(rtc, &time);
}

static int ssaiot_sc_rtc_alarm_irq_enable(struct device *dev,
					  unsigned int enabled)
{
	struct ssaiot_sc_rtc *rtc = dev_get_drvdata(dev);

	if (enabled)
		/* set_alarm already armed interrupt, nothing left to do */
		return 0;

	return ssaiot_sc_rtc_alarm_cancel(rtc);
}

/*
 * No set_time: the host has no path into the controller's calendar, which is
 * set from GNSS alone.
 */
static const struct rtc_class_ops ssaiot_sc_rtc_ops = {
	.read_time = ssaiot_sc_rtc_read_time,
	.read_alarm = ssaiot_sc_rtc_read_alarm,
	.set_alarm = ssaiot_sc_rtc_set_alarm,
	.alarm_irq_enable = ssaiot_sc_rtc_alarm_irq_enable,
};

static irqreturn_t ssaiot_sc_rtc_irq(int irq, void *data)
{
	struct ssaiot_sc_rtc *rtc = data;

	rtc_update_irq(rtc->rtc, 1, RTC_AF | RTC_IRQF);

	return IRQ_HANDLED;
}

static int ssaiot_sc_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssaiot_sc_rtc *rtc;
	int ret;

	/* the mfd cell has no dedicated dt node, reuse parent */
	dev->of_node = dev->parent->of_node;

	rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->sc = dev_get_drvdata(dev->parent);
	platform_set_drvdata(pdev, rtc);

	rtc->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(rtc->rtc))
		return PTR_ERR(rtc->rtc);

	rtc->rtc->ops = &ssaiot_sc_rtc_ops;

	/* the calendar carries a two digit year against a 2000 epoch */
	rtc->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	rtc->rtc->range_max = RTC_TIMESTAMP_END_2099;

	rtc->irq = platform_get_irq_byname(pdev, "alarm");
	if (rtc->irq < 0)
		return rtc->irq;

	/* request threaded irq to allow long i2c transfers while processing */
	ret = devm_request_threaded_irq(dev, rtc->irq, NULL, ssaiot_sc_rtc_irq,
					IRQF_ONESHOT, dev_name(dev), rtc);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request alarm irq.\n");

	device_init_wakeup(dev, true);

	return devm_rtc_register_device(rtc->rtc);
}

static int ssaiot_sc_rtc_suspend(struct device *dev)
{
	struct ssaiot_sc_rtc *rtc = dev_get_drvdata(dev);

	/* check if device is set as wakeup source */
	if (!device_may_wakeup(dev))
		return 0;

	/* enable irq wakeup */
	return enable_irq_wake(rtc->irq);
}

static int ssaiot_sc_rtc_resume(struct device *dev)
{
	struct ssaiot_sc_rtc *rtc = dev_get_drvdata(dev);

	/* check if device was set as wakeup source */
	if (!device_may_wakeup(dev))
		return 0;

	/* disable irq wakeup */
	return disable_irq_wake(rtc->irq);
}

static DEFINE_SIMPLE_DEV_PM_OPS(ssaiot_sc_rtc_pm_ops, ssaiot_sc_rtc_suspend,
				ssaiot_sc_rtc_resume);

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
		.pm = pm_sleep_ptr(&ssaiot_sc_rtc_pm_ops),
	},
	.probe = ssaiot_sc_rtc_probe,
	.id_table = ssaiot_sc_rtc_id_table,
};
module_platform_driver(ssaiot_sc_rtc_driver);

MODULE_AUTHOR("Josua Mayer");
MODULE_DESCRIPTION("SolidRun SolidSense AIOT Board System Controller RTC Driver");
MODULE_LICENSE("GPL v2");
