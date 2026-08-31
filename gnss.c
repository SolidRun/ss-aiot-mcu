// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller GNSS Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/gnss.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include "ssaiot_sc.h"

/*
 * The controller pads the NMEA payload to a constant size rather than
 * shortening it, so a read is always for exactly this many bytes. Asking for
 * more leaves the controller stretching SCL with nothing left to send, which
 * wedges the bus until its watchdog fires some ten seconds later.
 */
#define SSAIOT_SC_GNSS_PAYLOAD_LEN	32

/*
 * The controller's queue holds roughly one second of sentences, and the module
 * emits its batch in a once-per-second burst. Drain the queue, then wait well
 * inside that second so nothing is overwritten before the next pass.
 */
#define SSAIOT_SC_GNSS_IDLE_MS		100

struct ssaiot_sc_gnss {
	struct ssaiot_sc_priv *sc;
	struct gnss_device *gdev;
	struct delayed_work work;
};

/**
 * ssaiot_sc_gnss_poll() - Pump sentences out of the controller
 * @work: Poll work of the GNSS sub-device
 *
 * Queued only while the character device is open, so an unused receiver costs
 * no bus traffic. The controller cannot signal new data, so the stream has to
 * be polled: one read per run, re-queued without delay for as long as payloads
 * come back full, and backed off as soon as one arrives padded - or the queue
 * reports itself empty.
 *
 * Re-queueing rather than looping keeps each read a separate work item, so a
 * busy receiver yields to anything else pending on the controller's ordered
 * poll queue instead of holding the bus back to back.
 */
static void ssaiot_sc_gnss_poll(struct work_struct *work)
{
	struct ssaiot_sc_gnss *gnss = container_of(to_delayed_work(work),
						   struct ssaiot_sc_gnss, work);
	u8 buf[SSAIOT_SC_GNSS_PAYLOAD_LEN];
	unsigned int delay_ms = SSAIOT_SC_GNSS_IDLE_MS;
	u8 status;
	int ret;

	/*
	 * An empty queue is reported in the status, not in the length. A failed
	 * transfer is handled the same way: there is nothing to hand over, so
	 * come back on the next pass and try again.
	 */
	ret = ssaiot_sc_xfer(gnss->sc, SSAIOT_SC_CMD_SENSOR_READ,
			     SSAIOT_SC_SENSOR_GPS, NULL, 0,
			     buf, sizeof(buf), &status);
	if (!ret && status == SSAIOT_SC_STATUS_OK) {
		size_t len;

		/*
		 * Sentences are queued whole and packed back to back, so the
		 * payload is handed over verbatim - exactly as it would arrive
		 * on a serial port.
		 *
		 * What is not handed over is the padding. The controller fills
		 * the rest of a payload with newlines when its queue runs dry
		 * part way through, which reaches userspace as empty lines. A
		 * framer discards those anyway, so drop them here rather than
		 * carry them through the fifo and a read().
		 *
		 * Sentences end "\r\n" and are packed back to back, so the byte
		 * after a sentence's newline is either the '$' of the next one
		 * or padding. A newline preceded by a newline is therefore
		 * always padding, and walking back while that holds stops on
		 * the newline that closes the last sentence.
		 */
		for (len = SSAIOT_SC_GNSS_PAYLOAD_LEN;
		     len >= 2 && buf[len - 1] == '\n' && buf[len - 2] == '\n';
		     len--);

		ret = gnss_insert_raw(gnss->gdev, buf, len);
		if (ret < (int)len)
			dev_warn_ratelimited(&gnss->gdev->dev,
					     "dropped %d bytes, reader is too slow.\n",
					     (int)len - ret);

		/*
		 * Padding is only appended once the controller's queue runs
		 * dry, so a short payload says there is nothing left to collect
		 * and the next read would only report an empty queue. A full
		 * payload says the opposite - come straight back for the rest.
		 */
		if (len == SSAIOT_SC_GNSS_PAYLOAD_LEN)
			delay_ms = 0;
	}

	queue_delayed_work(gnss->sc->wq, &gnss->work,
			   msecs_to_jiffies(delay_ms));
}

static int ssaiot_sc_gnss_open(struct gnss_device *gdev)
{
	struct ssaiot_sc_gnss *gnss = gnss_get_drvdata(gdev);

	queue_delayed_work(gnss->sc->wq, &gnss->work, 0);

	return 0;
}

static void ssaiot_sc_gnss_close(struct gnss_device *gdev)
{
	struct ssaiot_sc_gnss *gnss = gnss_get_drvdata(gdev);

	/* safe against the work re-queueing itself */
	cancel_delayed_work_sync(&gnss->work);
}

/*
 * No write_raw: the controller passes the module's output through but offers no
 * path back to it, so the device is read only. The GNSS core reports -EIO for
 * writes when the callback is absent.
 */
static const struct gnss_operations ssaiot_sc_gnss_ops = {
	.open = ssaiot_sc_gnss_open,
	.close = ssaiot_sc_gnss_close,
};

static int ssaiot_sc_gnss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssaiot_sc_gnss *gnss;
	struct gnss_device *gdev;
	int ret;

	/* the mfd cell has no dedicated dt node, reuse parent */
	dev->of_node = dev->parent->of_node;

	gnss = devm_kzalloc(dev, sizeof(*gnss), GFP_KERNEL);
	if (!gnss)
		return -ENOMEM;

	gnss->sc = dev_get_drvdata(dev->parent);
	INIT_DELAYED_WORK(&gnss->work, ssaiot_sc_gnss_poll);

	gdev = gnss_allocate_device(dev);
	if (!gdev)
		return -ENOMEM;

	gdev->type = GNSS_TYPE_NMEA;
	gdev->ops = &ssaiot_sc_gnss_ops;
	gnss->gdev = gdev;
	gnss_set_drvdata(gdev, gnss);

	ret = gnss_register_device(gdev);
	if (ret) {
		gnss_put_device(gdev);
		return dev_err_probe(dev, ret, "Failed to register gnss device.\n");
	}

	platform_set_drvdata(pdev, gnss);

	return 0;
}

static int ssaiot_sc_gnss_remove(struct platform_device *pdev)
{
	struct ssaiot_sc_gnss *gnss = platform_get_drvdata(pdev);

	gnss_deregister_device(gnss->gdev);
	gnss_put_device(gnss->gdev);

	return 0;
}

/*
 * The id must match the MFD cell name and is capped at PLATFORM_NAME_SIZE,
 * so it stays short. Since an id table suppresses the driver name fallback in
 * platform_match(), the driver name itself is free to be descriptive.
 */
static const struct platform_device_id ssaiot_sc_gnss_id_table[] = {
	{ "ssaiot-sc-gnss", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, ssaiot_sc_gnss_id_table);

static struct platform_driver ssaiot_sc_gnss_driver = {
	.driver = {
		.name = "solidsense-aiot-system-controller-gnss",
	},
	.probe = ssaiot_sc_gnss_probe,
	.remove = ssaiot_sc_gnss_remove,
	.id_table = ssaiot_sc_gnss_id_table,
};
module_platform_driver(ssaiot_sc_gnss_driver);

MODULE_AUTHOR("Josua Mayer");
MODULE_DESCRIPTION("SolidRun SolidSense AIOT Board System Controller GNSS Driver");
MODULE_LICENSE("GPL v2");
