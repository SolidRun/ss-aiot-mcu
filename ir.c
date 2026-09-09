// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Infrared Sensor Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "ssaiot_sc.h"

struct ssaiot_sc_ir_sample {
	__le32 timestamp; /* 25us per LSB, mcu timebase */
	__le16 presence; /* raw algorithm output */
	__le16 motion; /* raw algorithm output */
	__le16 tamb; /* raw, 100 LSB per degree celsius */
} __packed;

/* time unit of the controller's timebase, 25us/LSB */
#define SSAIOT_SC_IR_TICK_NS 25000

/* maximum infrared samples per read (payload size) */
#define SSAIOT_SC_IR_PER_READ 5

/* whole response, as read */
struct ssaiot_sc_ir_resp {
	__le32 now; /* 25us per LSB, mcu timebase when the read was decoded */
	struct ssaiot_sc_ir_sample sample[SSAIOT_SC_IR_PER_READ];
} __packed;

static_assert(sizeof(struct ssaiot_sc_ir_resp) <= SSAIOT_SC_RESP_MAX_DATA_LEN);

/* how long to wait before looking again once the stream has run dry */
#define SSAIOT_SC_IR_IDLE_MS 100

/* detectors the controller reports, and the channel address each event uses */
#define SSAIOT_SC_IR_EV_PRESENCE 0
#define SSAIOT_SC_IR_EV_MOTION 1

/*
 * Scan indices of the readings, and their positions in the scan below. The two
 * detectors number their channels the same way, so an event code and its
 * channel cannot drift apart.
 */
#define SSAIOT_SC_IR_SCAN_PRESENCE 0
#define SSAIOT_SC_IR_SCAN_MOTION 1
#define SSAIOT_SC_IR_SCAN_TAMB 2

/* driver private data */
struct ssaiot_sc_ir_priv {
	struct ssaiot_sc_priv *sc;
	struct iio_dev *indio_dev;
	struct delayed_work work;

	/* one scan includes presence, motion, ambient + timestamp */
	struct {
		s16 presence;
		s16 motion;
		s16 tamb;
		aligned_s64 timestamp;
	} scan;

	/* which detectors userspace asked to hear about */
	unsigned long events;
};

/* calculate record count from response size in bytes, after header */
static inline int ssaiot_sc_ir_records(u8 len, size_t hdr_len, size_t rec_size)
{
	if (len < hdr_len || (len - hdr_len) % rec_size)
		return -EPROTO;

	return (len - hdr_len) / rec_size;
}

/*
 * Convert a record's controller timestamp to CLOCK_REALTIME, given the host
 * time of the snapshot the same response opened with. The counter difference is
 * taken modulo 2^32 and read signed, so a wrap resolves to the shorter interval.
 */
static s64 ssaiot_sc_ir_timestamp(s64 ts_ref, __le32 now, __le32 stamp)
{
	s32 delta = (s32)(le32_to_cpu(stamp) - le32_to_cpu(now));

	return ts_ref + (s64)delta * SSAIOT_SC_IR_TICK_NS;
}

/*
 * Read captured samples once and push them to the buffer, then re-queue: with
 * no delay while responses come back full, otherwise after the idle interval.
 * Runs only while the buffer is enabled.
 */
static void ssaiot_sc_ir_poll(struct work_struct *work)
{
	struct ssaiot_sc_ir_priv *priv =
		container_of(to_delayed_work(work),
			     struct ssaiot_sc_ir_priv, work);
	unsigned int delay_ms = SSAIOT_SC_IR_IDLE_MS;
	struct ssaiot_sc_ir_resp resp;
	s64 ts_ref;
	u8 len;
	int n, i;

	n = ssaiot_sc_xfer(priv->sc, SSAIOT_SC_CMD_SENSOR_READ,
			   SSAIOT_SC_SENSOR_IR, NULL, 0,
			   (u8 *)&resp, sizeof(resp), &len, NULL, &ts_ref);
	if (!n)
		n = ssaiot_sc_ir_records(len, offsetof(typeof(resp), sample),
					 sizeof(resp.sample[0]));

	if (n < 0) {
		dev_warn_ratelimited(priv->sc->dev,
				     "infrared read failed: %d.\n", n);
	} else {
		for (i = 0; i < n; i++) {
			priv->scan.presence =
				(s16)le16_to_cpu(resp.sample[i].presence);
			priv->scan.motion =
				(s16)le16_to_cpu(resp.sample[i].motion);
			priv->scan.tamb =
				(s16)le16_to_cpu(resp.sample[i].tamb);

			iio_push_to_buffers_with_timestamp(priv->indio_dev,
					&priv->scan,
					ssaiot_sc_ir_timestamp(ts_ref, resp.now,
							resp.sample[i].timestamp));
		}

		/* a full response says the capture held at least this much more */
		if (n == ARRAY_SIZE(resp.sample))
			delay_ms = 0;
	}

	queue_delayed_work(priv->sc->wq, &priv->work, msecs_to_jiffies(delay_ms));
}

static int ssaiot_sc_ir_postenable(struct iio_dev *indio_dev)
{
	struct ssaiot_sc_ir_priv *priv = iio_priv(indio_dev);

	queue_delayed_work(priv->sc->wq, &priv->work, 0);

	return 0;
}

static int ssaiot_sc_ir_predisable(struct iio_dev *indio_dev)
{
	struct ssaiot_sc_ir_priv *priv = iio_priv(indio_dev);

	/* safe against the work re-queueing itself */
	cancel_delayed_work_sync(&priv->work);

	return 0;
}

static const struct iio_buffer_setup_ops ssaiot_sc_ir_setup_ops = {
	.postenable = ssaiot_sc_ir_postenable,
	.predisable = ssaiot_sc_ir_predisable,
};

/*
 * No IIO_CHAN_INFO_RAW: the protocol offers no reading of the current values,
 * only a queue that is served oldest first and consumed by being read. A
 * one-shot attribute over that would hand back a stale sample and destroy it on
 * the way past. The buffer is the whole interface; read one sample from it if
 * that is all that is wanted.
 *
 * Presence and motion carry no scale either. They are the differences between
 * two of the sensor's internal low-pass filters, so they have no unit and are
 * meaningful only against the detector thresholds.
 */
static int ssaiot_sc_ir_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		/* 100 LSB per degree against an abi in millidegrees */
		*val = 10;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

/* name the two proximity channels, which are otherwise told apart by index */
static int ssaiot_sc_ir_read_label(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan,
				   char *label)
{
	switch (chan->scan_index) {
	case SSAIOT_SC_IR_SCAN_PRESENCE:
		return sysfs_emit(label, "presence\n");

	case SSAIOT_SC_IR_SCAN_MOTION:
		return sysfs_emit(label, "motion\n");

	case SSAIOT_SC_IR_SCAN_TAMB:
		return sysfs_emit(label, "ambient\n");

	default:
		return -EINVAL;
	}
}

/* report one detector, if userspace is listening for it */
static irqreturn_t ssaiot_sc_ir_event(struct iio_dev *indio_dev,
				      unsigned int ev, u64 code)
{
	struct ssaiot_sc_ir_priv *priv = iio_priv(indio_dev);

	if (test_bit(ev, &priv->events))
		iio_push_event(indio_dev, code, iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static irqreturn_t ssaiot_sc_ir_presence_event(int irq, void *data)
{
	return ssaiot_sc_ir_event(data, SSAIOT_SC_IR_EV_PRESENCE,
				  IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,
						       SSAIOT_SC_IR_SCAN_PRESENCE,
						       IIO_EV_TYPE_THRESH,
						       IIO_EV_DIR_RISING));
}

static irqreturn_t ssaiot_sc_ir_activity_event(int irq, void *data)
{
	return ssaiot_sc_ir_event(data, SSAIOT_SC_IR_EV_MOTION,
				  IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,
						       SSAIOT_SC_IR_SCAN_MOTION,
						       IIO_EV_TYPE_THRESH,
						       IIO_EV_DIR_RISING));
}

/*
 * The detectors run in the controller and cannot be turned off from here, so
 * these gate reporting rather than the hardware. The channel address says which
 * detector an attribute belongs to.
 */
static int ssaiot_sc_ir_read_event_config(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir)
{
	struct ssaiot_sc_ir_priv *priv = iio_priv(indio_dev);

	return test_bit(chan->address, &priv->events);
}

static int ssaiot_sc_ir_write_event_config(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   enum iio_event_type type,
					   enum iio_event_direction dir,
					   int state)
{
	struct ssaiot_sc_ir_priv *priv = iio_priv(indio_dev);

	assign_bit(chan->address, &priv->events, state);

	return 0;
}

static const struct iio_info ssaiot_sc_ir_info = {
	.read_raw = ssaiot_sc_ir_read_raw,
	.read_label = ssaiot_sc_ir_read_label,
	.read_event_config = ssaiot_sc_ir_read_event_config,
	.write_event_config = ssaiot_sc_ir_write_event_config,
};

/* the thresholds are firmware configured, so only reporting can be turned off */
#define SSAIOT_SC_IR_EVENT_SPEC(_type, _dir) {				\
	.type = _type,							\
	.dir = _dir,							\
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),			\
}

static const struct iio_event_spec ssaiot_sc_ir_presence_event_spec[] = {
	SSAIOT_SC_IR_EVENT_SPEC(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
};

static const struct iio_event_spec ssaiot_sc_ir_motion_event_spec[] = {
	SSAIOT_SC_IR_EVENT_SPEC(IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
};

#define SSAIOT_SC_IR_DETECTOR_CHANNEL(_name, _spec) {			\
	.type = IIO_PROXIMITY,						\
	.indexed = 1,							\
	.channel = SSAIOT_SC_IR_SCAN_##_name,				\
	.address = SSAIOT_SC_IR_EV_##_name,				\
	.scan_index = SSAIOT_SC_IR_SCAN_##_name,			\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_CPU,					\
	},								\
	.event_spec = _spec,						\
	.num_event_specs = ARRAY_SIZE(_spec),				\
}

static const struct iio_chan_spec ssaiot_sc_ir_channels[] = {
	SSAIOT_SC_IR_DETECTOR_CHANNEL(PRESENCE, ssaiot_sc_ir_presence_event_spec),
	SSAIOT_SC_IR_DETECTOR_CHANNEL(MOTION, ssaiot_sc_ir_motion_event_spec),
	{
		/* the sensor's own package, not the air in front of it */
		.type = IIO_TEMP,
		.scan_index = SSAIOT_SC_IR_SCAN_TAMB,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),
	},
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

/* one record carries all three readings, so scan them together */
static const unsigned long ssaiot_sc_ir_scan_masks[] = {
	BIT(SSAIOT_SC_IR_SCAN_PRESENCE) | BIT(SSAIOT_SC_IR_SCAN_MOTION) |
	BIT(SSAIOT_SC_IR_SCAN_TAMB),
	0
};

static int ssaiot_sc_ir_request_event(struct device *dev,
				      struct iio_dev *indio_dev,
				      const char *name,
				      irq_handler_t handler)
{
	int irq;

	irq = platform_get_irq_byname(to_platform_device(dev), name);
	if (irq < 0)
		return irq;

	/* nested and threaded, so the handler runs in the demux thread */
	return devm_request_threaded_irq(dev, irq, NULL, handler, IRQF_ONESHOT,
					 name, indio_dev);
}

static int ssaiot_sc_ir_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssaiot_sc_ir_priv *priv;
	struct iio_dev *indio_dev;
	int ret;

	/* the mfd cell has no dedicated dt node, reuse parent */
	dev->of_node = dev->parent->of_node;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	priv->sc = dev_get_drvdata(dev->parent);
	priv->indio_dev = indio_dev;
	platform_set_drvdata(pdev, priv);
	INIT_DELAYED_WORK(&priv->work, ssaiot_sc_ir_poll);

	indio_dev->name = "ssaiot-sc-ir";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ssaiot_sc_ir_info;
	indio_dev->channels = ssaiot_sc_ir_channels;
	indio_dev->num_channels = ARRAY_SIZE(ssaiot_sc_ir_channels);
	indio_dev->available_scan_masks = ssaiot_sc_ir_scan_masks;

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev,
					  &ssaiot_sc_ir_setup_ops);
	if (ret)
		return ret;

	ret = ssaiot_sc_ir_request_event(dev, indio_dev, "presence",
					 ssaiot_sc_ir_presence_event);
	if (ret)
		return ret;

	ret = ssaiot_sc_ir_request_event(dev, indio_dev, "activity",
					 ssaiot_sc_ir_activity_event);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

/* prepare for shutdown, i.e. release the bus and disable interrupts */
static void ssaiot_sc_ir_shutdown(struct platform_device *pdev)
{
	struct ssaiot_sc_ir_priv *priv = platform_get_drvdata(pdev);

	/* stop work to release the bus */
	cancel_delayed_work_sync(&priv->work);
}

/*
 * The id must match the MFD cell name and is capped at PLATFORM_NAME_SIZE,
 * so it stays short. Since an id table suppresses the driver name fallback in
 * platform_match(), the driver name itself is free to be descriptive.
 */
static const struct platform_device_id ssaiot_sc_ir_id_table[] = {
	{ "ssaiot-sc-ir", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, ssaiot_sc_ir_id_table);

static struct platform_driver ssaiot_sc_ir_driver = {
	.driver = {
		.name = "solidsense-aiot-system-controller-ir",
	},
	.probe = ssaiot_sc_ir_probe,
	.shutdown = ssaiot_sc_ir_shutdown,
	.id_table = ssaiot_sc_ir_id_table,
};
module_platform_driver(ssaiot_sc_ir_driver);

MODULE_AUTHOR("Josua Mayer");
MODULE_DESCRIPTION("SolidRun SolidSense AIOT Board System Controller Infrared Sensor Driver");
MODULE_LICENSE("GPL v2");
