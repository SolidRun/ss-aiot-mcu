// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Infrared Sensor Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/bitmap.h>
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
	__le16 tobject; /* raw, scaled by SENSITIVITY below */
} __packed;

struct ssaiot_sc_ir_config {
	u8 flags;
	u8 sensitivity; /* of tobject in default gain mode, in units of 16 LSB/degC */
} __packed;

#define SSAIOT_SC_IR_WIDE_GAIN		BIT(0)

/* unit of the sensitivity field in default gain mode */
#define SSAIOT_SC_IR_SENS_PER_LSB	16
/* unit of the sensitivity field in wide gain mode */
#define SSAIOT_SC_IR_SENS_PER_LSB_WIDE	2

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
enum ssaiot_sc_ir_ev {
	SSAIOT_SC_IR_EV_PRESENCE = 0,
	SSAIOT_SC_IR_EV_MOTION,
	SSAIOT_SC_IR_EV_MAX,
};

/*
 * Scan indices of the readings, and their positions in the scan below. The two
 * detectors number their channels the same way, so an event code and its
 * channel cannot drift apart.
 */
#define SSAIOT_SC_IR_SCAN_PRESENCE 0
#define SSAIOT_SC_IR_SCAN_MOTION 1
#define SSAIOT_SC_IR_SCAN_TAMB 2
#define SSAIOT_SC_IR_SCAN_TOBJ 3

/* driver private data */
struct ssaiot_sc_ir_priv {
	struct ssaiot_sc_priv *sc;
	struct iio_dev *indio_dev;
	struct delayed_work work;

	/* one scan includes presence, motion, ambient, object + timestamp */
	struct {
		s16 presence;
		s16 motion;
		s16 tamb;
		s16 tobject;
		aligned_s64 timestamp;
	} scan;

	/* the controller's own configuration, which the object scale follows */
	struct ssaiot_sc_ir_config config;

	/* which detectors userspace asked to hear about, and their interrupts */
	unsigned long events;
	int event_irq[SSAIOT_SC_IR_EV_MAX];
};

/*
 * Calculate record count from the response's own length, after the header. The
 * controller may answer with more than was read - a later firmware batching
 * more records - so the count is capped at what the buffer holds; the rest is
 * dropped when the next command goes out.
 */
static inline int ssaiot_sc_ir_records(u8 data_len, size_t rx_size,
				       size_t hdr_len, size_t rec_size)
{
	if (data_len < hdr_len || (data_len - hdr_len) % rec_size)
		return -EPROTO;

	return (min_t(size_t, data_len, rx_size) - hdr_len) / rec_size;
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
	u8 data_len;
	int n, i;

	n = ssaiot_sc_xfer(priv->sc, SSAIOT_SC_CMD_SENSOR_READ,
			   SSAIOT_SC_SENSOR_IR, NULL, 0,
			   (u8 *)&resp, sizeof(resp), &data_len, NULL, &ts_ref);
	if (!n)
		n = ssaiot_sc_ir_records(data_len, sizeof(resp),
					 offsetof(typeof(resp), sample),
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
			priv->scan.tobject =
				(s16)le16_to_cpu(resp.sample[i].tobject);

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
 * Presence and motion are differences between low-pass filters over the object
 * reading, so they share its unit. Neither carries a scale: a proximity channel
 * reads as metres once one is applied. Both temperatures carry one - the
 * ambient's fixed by the protocol, the object's following the part and the gain
 * mode and so coming from the controller.
 */
static int ssaiot_sc_ir_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	struct ssaiot_sc_ir_priv *priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->channel2) {
		case IIO_MOD_TEMP_AMBIENT:
			/* 100 LSB per degree against an abi in millidegrees */
			*val = 10;
			return IIO_VAL_INT;

		case IIO_MOD_TEMP_OBJECT:
			/* Millidegrees per LSB, scale depends on gain mode */
			*val = 1000;
			*val2 = priv->config.sensitivity;
			if (priv->config.flags & SSAIOT_SC_IR_WIDE_GAIN)
				*val2 *= SSAIOT_SC_IR_SENS_PER_LSB_WIDE;
			else
				*val2 *= SSAIOT_SC_IR_SENS_PER_LSB;
			return IIO_VAL_FRACTIONAL;

		default:
			return -EINVAL;
		}

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

	case SSAIOT_SC_IR_SCAN_TOBJ:
		return sysfs_emit(label, "object\n");

	default:
		return -EINVAL;
	}
}

/* report one detector */
static irqreturn_t ssaiot_sc_ir_event(struct iio_dev *indio_dev, u64 code)
{
	iio_push_event(indio_dev, code, iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static irqreturn_t ssaiot_sc_ir_presence_event(int irq, void *data)
{
	return ssaiot_sc_ir_event(data, IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,
							     SSAIOT_SC_IR_SCAN_PRESENCE,
							     IIO_EV_TYPE_THRESH,
							     IIO_EV_DIR_RISING));
}

static irqreturn_t ssaiot_sc_ir_activity_event(int irq, void *data)
{
	return ssaiot_sc_ir_event(data, IIO_UNMOD_EVENT_CODE(IIO_PROXIMITY,
							     SSAIOT_SC_IR_SCAN_MOTION,
							     IIO_EV_TYPE_THRESH,
							     IIO_EV_DIR_RISING));
}

/*
 * Enabling an event unmasks its interrupt, which the demultiplexer forwards to
 * the controller's own enable mask, so a disabled detector is not reported at
 * all. The channel address says which detector an attribute belongs to.
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

	/*
	 * The irq depth is a refcount and sysfs does not filter repeated writes,
	 * so the bit both records the state and claims the right to change it.
	 * Nothing serialises two writers of the same attribute.
	 */
	if (state) {
		if (test_and_set_bit(chan->address, &priv->events))
			return 0;

		enable_irq(priv->event_irq[chan->address]);
	} else {
		if (!test_and_clear_bit(chan->address, &priv->events))
			return 0;

		disable_irq(priv->event_irq[chan->address]);
	}

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
		.modified = 1,
		.channel2 = IIO_MOD_TEMP_AMBIENT,
		.scan_index = SSAIOT_SC_IR_SCAN_TAMB,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		/*
		 * Radiation from the field of view, as a temperature difference
		 * against the sensor's own package, which it takes for the
		 * room's. Presence and motion are filters over this signal.
		 */
		.type = IIO_TEMP,
		.modified = 1,
		.channel2 = IIO_MOD_TEMP_OBJECT,
		.scan_index = SSAIOT_SC_IR_SCAN_TOBJ,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),
	},
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

/* one record carries all three readings, so scan them together */
static const unsigned long ssaiot_sc_ir_scan_masks[] = {
	BIT(SSAIOT_SC_IR_SCAN_PRESENCE) | BIT(SSAIOT_SC_IR_SCAN_MOTION) |
	BIT(SSAIOT_SC_IR_SCAN_TAMB) | BIT(SSAIOT_SC_IR_SCAN_TOBJ),
	0
};

static int ssaiot_sc_ir_request_event(struct device *dev,
				      struct iio_dev *indio_dev,
				      enum ssaiot_sc_ir_ev ev,
				      const char *name, irq_handler_t handler)
{
	struct ssaiot_sc_ir_priv *priv = iio_priv(indio_dev);
	int irq, ret;

	irq = platform_get_irq_byname(to_platform_device(dev), name);
	if (irq < 0)
		return irq;

	/*
	 * Nested and threaded, so the handler runs in the demux thread. Left
	 * masked because the unmask reaches the controller's own enable mask:
	 * the event attribute is what turns the detector on.
	 */
	ret = devm_request_threaded_irq(dev, irq, NULL, handler,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					name, indio_dev);
	if (ret)
		return ret;

	priv->event_irq[ev] = irq;

	return 0;
}

/* read controller active configuration */
static int ssaiot_sc_ir_read_config(struct device *dev,
				    struct ssaiot_sc_ir_priv *priv)
{
	u8 data_len;
	int ret;

	ret = ssaiot_sc_xfer(priv->sc, SSAIOT_SC_CMD_SENSOR_CONFIG,
			     SSAIOT_SC_SENSOR_IR, NULL, 0,
			     (u8 *)&priv->config, sizeof(priv->config),
			     &data_len, NULL, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read infrared config.\n");

	if (data_len < sizeof(priv->config))
		return dev_err_probe(dev, -EPROTO,
				     "Controller reports %u bytes of infrared config, need %zu.\n",
				     data_len, sizeof(priv->config));

	if (!priv->config.sensitivity)
		return dev_err_probe(dev, -EPROTO,
				     "Controller reports invalid infrared sensitivity.\n");

	return 0;
}

static int ssaiot_sc_ir_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssaiot_sc_ir_priv *priv;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	priv->sc = dev_get_drvdata(dev->parent);
	priv->indio_dev = indio_dev;
	platform_set_drvdata(pdev, priv);
	INIT_DELAYED_WORK(&priv->work, ssaiot_sc_ir_poll);

	ret = ssaiot_sc_ir_read_config(dev, priv);
	if (ret)
		return ret;

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

	device_init_wakeup(dev, true);

	ret = ssaiot_sc_irq_claim(priv->sc, SSAIOT_SC_INT_SRC_IR,
				  device_may_wakeup(dev));
	if (ret)
		return ret;

	ret = ssaiot_sc_ir_request_event(dev, indio_dev, SSAIOT_SC_IR_EV_PRESENCE,
					 "presence", ssaiot_sc_ir_presence_event);
	if (ret)
		return ret;

	ret = ssaiot_sc_ir_request_event(dev, indio_dev, SSAIOT_SC_IR_EV_MOTION,
					 "activity", ssaiot_sc_ir_activity_event);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

/* prepare for shutdown, i.e. release the bus and apply the wake-up policy */
static void ssaiot_sc_ir_shutdown(struct platform_device *pdev)
{
	struct ssaiot_sc_ir_priv *priv = platform_get_drvdata(pdev);

	/* stop work to release the bus */
	cancel_delayed_work_sync(&priv->work);

	/* apply wake-up policy */
	ssaiot_sc_irq_set_poweron(priv->sc, SSAIOT_SC_INT_SRC_IR,
				  device_may_wakeup(&pdev->dev));
}

/*
 * Only the detectors userspace enabled can assert, so those are the ones to
 * arm. Their wake references have to be given back one for one, which holds
 * because userspace is frozen before this runs and cannot change the set.
 */
static int ssaiot_sc_ir_suspend(struct device *dev)
{
	struct ssaiot_sc_ir_priv *priv = dev_get_drvdata(dev);
	unsigned int ev, armed;
	int ret;

	/* check if device is set as wakeup source */
	if (!device_may_wakeup(dev))
		return 0;

	/* enable irq wakeup */
	for_each_set_bit(ev, &priv->events, SSAIOT_SC_IR_EV_MAX) {
		ret = enable_irq_wake(priv->event_irq[ev]);
		if (ret)
			goto err;
	}

	return 0;

err:
	/* a device whose suspend failed is not resumed, so unwind here */
	for_each_set_bit(armed, &priv->events, ev)
		disable_irq_wake(priv->event_irq[armed]);

	return ret;
}

static int ssaiot_sc_ir_resume(struct device *dev)
{
	struct ssaiot_sc_ir_priv *priv = dev_get_drvdata(dev);
	unsigned int ev;

	/* check if device was set as wakeup source */
	if (!device_may_wakeup(dev))
		return 0;

	/* disable irq wakeup */
	for_each_set_bit(ev, &priv->events, SSAIOT_SC_IR_EV_MAX)
		disable_irq_wake(priv->event_irq[ev]);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ssaiot_sc_ir_pm_ops, ssaiot_sc_ir_suspend,
				ssaiot_sc_ir_resume);

/* a cell with a dt node reports an of: modalias, which is what autoloads this */
static const struct of_device_id ssaiot_sc_ir_of_match[] = {
	{ .compatible = "solidrun,solidsense-aiot-system-controller-ir" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ssaiot_sc_ir_of_match);

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
		.of_match_table = ssaiot_sc_ir_of_match,
		.pm = pm_sleep_ptr(&ssaiot_sc_ir_pm_ops),
	},
	.probe = ssaiot_sc_ir_probe,
	.shutdown = ssaiot_sc_ir_shutdown,
	.id_table = ssaiot_sc_ir_id_table,
};
module_platform_driver(ssaiot_sc_ir_driver);

MODULE_AUTHOR("Josua Mayer");
MODULE_DESCRIPTION("SolidRun SolidSense AIOT Board System Controller Infrared Sensor Driver");
MODULE_LICENSE("GPL v2");
