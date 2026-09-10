// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Accelerometer Driver
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
#include <linux/workqueue.h>

#include "ssaiot_sc.h"

struct ssaiot_sc_accel_sample {
	__le32 timestamp; /* 25us per LSB, mcu timebase */
	__le16 x; /* raw, 0.061 mg/LSB at +-2g */
	__le16 y; /* raw, 0.061 mg/LSB at +-2g */
	__le16 z; /* raw, 0.061 mg/LSB at +-2g */
} __packed;

struct ssaiot_sc_accel_temp {
	__le32 timestamp;
	__le16 temp; /* raw, 256 LSB per degree celsius, 0 is 25 degrees */
} __packed;

/* time unit of the controller's timebase, 25us/LSB */
#define SSAIOT_SC_ACCEL_TICK_NS 25000

/* maximum accelerometer motion data samples per read (payload size) */
#define SSAIOT_SC_ACCEL_MOTION_PER_READ 8
/* maximum accelerometer temperature data samples per read (payload size) */
#define SSAIOT_SC_ACCEL_TEMP_PER_READ 1

/* whole responses, as read */
struct ssaiot_sc_accel_motion_resp {
	__le32 now; /* 25us per LSB, mcu timebase when the read was decoded */
	struct ssaiot_sc_accel_sample sample[SSAIOT_SC_ACCEL_MOTION_PER_READ];
} __packed;

struct ssaiot_sc_accel_temp_resp {
	__le32 now;
	struct ssaiot_sc_accel_temp sample[SSAIOT_SC_ACCEL_TEMP_PER_READ];
} __packed;

static_assert(sizeof(struct ssaiot_sc_accel_motion_resp) <= SSAIOT_SC_RESP_MAX_DATA_LEN);
static_assert(sizeof(struct ssaiot_sc_accel_temp_resp) <= SSAIOT_SC_RESP_MAX_DATA_LEN);

/* how long to wait before looking again once a stream has run dry */
#define SSAIOT_SC_ACCEL_MOTION_IDLE_MS 100
#define SSAIOT_SC_ACCEL_TEMP_IDLE_MS 1000

/* detectors the controller reports, and the channel address each event uses */
enum ssaiot_sc_accel_ev {
	SSAIOT_SC_ACCEL_EV_MOTION = 0,
	SSAIOT_SC_ACCEL_EV_TILT,
	SSAIOT_SC_ACCEL_EV_FREEFALL,
	SSAIOT_SC_ACCEL_EV_MAX,
};

/* scan indices of the axes, and their positions in the scan below */
#define SSAIOT_SC_ACCEL_X 0
#define SSAIOT_SC_ACCEL_Y 1
#define SSAIOT_SC_ACCEL_Z 2

/* driver private data, shared by both iio devices */
struct ssaiot_sc_accel_priv {
	struct ssaiot_sc_priv *sc;

	/* the motion stream, one scan includes 3 axis + timestamp */
	struct iio_dev *motion_iio;
	struct delayed_work motion_work;
	struct {
		s16 axis[3];
		aligned_s64 timestamp;
	} motion_scan;

	/* the temperature stream, one scan includes temperature + timestamp */
	struct iio_dev *temp_iio;
	struct delayed_work temp_work;
	struct {
		s16 temp;
		aligned_s64 timestamp;
	} temp_scan;

	/* which detectors userspace asked to hear about, and their interrupts */
	unsigned long events;
	int event_irq[SSAIOT_SC_ACCEL_EV_MAX];
};

/* each iio device keeps a pointer to it in the private area iio allocates */
static struct ssaiot_sc_accel_priv *ssaiot_sc_accel_iio_priv(struct iio_dev *indio_dev)
{
	return *(struct ssaiot_sc_accel_priv **)iio_priv(indio_dev);
}

/* calculate record count from response size in bytes, after header */
static inline int ssaiot_sc_accel_records(u8 len, size_t hdr_len, size_t rec_size)
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
static s64 ssaiot_sc_accel_timestamp(s64 ts_ref, __le32 now, __le32 stamp)
{
	s32 delta = (s32)(le32_to_cpu(stamp) - le32_to_cpu(now));

	return ts_ref + (s64)delta * SSAIOT_SC_ACCEL_TICK_NS;
}

/*
 * Read captured samples once and push them to the buffer, then re-queue: with
 * no delay while responses come back full, otherwise after the idle interval.
 * Runs only while the buffer is enabled.
 */
static void ssaiot_sc_accel_motion_poll(struct work_struct *work)
{
	struct ssaiot_sc_accel_priv *priv =
		container_of(to_delayed_work(work),
			     struct ssaiot_sc_accel_priv, motion_work);
	unsigned int delay_ms = SSAIOT_SC_ACCEL_MOTION_IDLE_MS;
	struct ssaiot_sc_accel_motion_resp resp;
	s64 ts_ref;
	u8 len;
	int n, i;

	n = ssaiot_sc_xfer(priv->sc, SSAIOT_SC_CMD_SENSOR_READ,
			   SSAIOT_SC_SENSOR_ACCEL_MOTION, NULL, 0,
			   (u8 *)&resp, sizeof(resp), &len, NULL, &ts_ref);
	if (!n)
		n = ssaiot_sc_accel_records(len, offsetof(typeof(resp), sample),
					    sizeof(resp.sample[0]));

	if (n < 0) {
		dev_warn_ratelimited(priv->sc->dev,
				     "motion read failed: %d.\n", n);
	} else {
		for (i = 0; i < n; i++) {
			priv->motion_scan.axis[SSAIOT_SC_ACCEL_X] =
				(s16)le16_to_cpu(resp.sample[i].x);
			priv->motion_scan.axis[SSAIOT_SC_ACCEL_Y] =
				(s16)le16_to_cpu(resp.sample[i].y);
			priv->motion_scan.axis[SSAIOT_SC_ACCEL_Z] =
				(s16)le16_to_cpu(resp.sample[i].z);

			iio_push_to_buffers_with_timestamp(priv->motion_iio,
					&priv->motion_scan,
					ssaiot_sc_accel_timestamp(ts_ref, resp.now,
							resp.sample[i].timestamp));
		}

		/* a full response says the capture held at least this much more */
		if (n == ARRAY_SIZE(resp.sample))
			delay_ms = 0;
	}

	queue_delayed_work(priv->sc->wq, &priv->motion_work,
			   msecs_to_jiffies(delay_ms));
}

static int ssaiot_sc_accel_motion_postenable(struct iio_dev *indio_dev)
{
	struct ssaiot_sc_accel_priv *priv = ssaiot_sc_accel_iio_priv(indio_dev);

	queue_delayed_work(priv->sc->wq, &priv->motion_work, 0);

	return 0;
}

static int ssaiot_sc_accel_motion_predisable(struct iio_dev *indio_dev)
{
	struct ssaiot_sc_accel_priv *priv = ssaiot_sc_accel_iio_priv(indio_dev);

	/* safe against the work re-queueing itself */
	cancel_delayed_work_sync(&priv->motion_work);

	return 0;
}

static const struct iio_buffer_setup_ops ssaiot_sc_accel_motion_setup_ops = {
	.postenable = ssaiot_sc_accel_motion_postenable,
	.predisable = ssaiot_sc_accel_motion_predisable,
};

/*
 * No IIO_CHAN_INFO_RAW: the protocol offers no reading of the current
 * acceleration, only a queue that is served oldest first and consumed by being
 * read. A one-shot attribute over that would hand back a stale sample and
 * destroy it on the way past. The buffer is the whole interface; read one
 * sample from it if that is all that is wanted.
 */
static int ssaiot_sc_accel_motion_read_raw(struct iio_dev *indio_dev,
					   struct iio_chan_spec const *chan,
					   int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		/*
		 * Full scale +-2g over a signed 16 bit reading, so one LSB is
		 * 2 * 9.80665 / 32768 m/s^2.
		 */
		*val = 0;
		*val2 = 598550;
		return IIO_VAL_INT_PLUS_NANO;

	default:
		return -EINVAL;
	}
}

/* report one detector */
static irqreturn_t ssaiot_sc_accel_event(struct iio_dev *indio_dev, u64 code)
{
	iio_push_event(indio_dev, code, iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static irqreturn_t ssaiot_sc_accel_motion_event(int irq, void *data)
{
	return ssaiot_sc_accel_event(data, IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							      IIO_MOD_X_OR_Y_OR_Z,
							      IIO_EV_TYPE_MAG_ADAPTIVE,
							      IIO_EV_DIR_RISING));
}

static irqreturn_t ssaiot_sc_accel_tilt_event(int irq, void *data)
{
	return ssaiot_sc_accel_event(data, IIO_UNMOD_EVENT_CODE(IIO_INCLI, 0,
								IIO_EV_TYPE_CHANGE,
								IIO_EV_DIR_EITHER));
}

static irqreturn_t ssaiot_sc_accel_freefall_event(int irq, void *data)
{
	return ssaiot_sc_accel_event(data, IIO_MOD_EVENT_CODE(IIO_ACCEL, 0,
							      IIO_MOD_X_AND_Y_AND_Z,
							      IIO_EV_TYPE_MAG,
							      IIO_EV_DIR_FALLING));
}

/*
 * Enabling an event unmasks its interrupt, which the demultiplexer forwards to
 * the controller's own enable mask, so a disabled detector is not reported at
 * all. The channel address says which detector an attribute belongs to.
 */
static int ssaiot_sc_accel_read_event_config(struct iio_dev *indio_dev,
					     const struct iio_chan_spec *chan,
					     enum iio_event_type type,
					     enum iio_event_direction dir)
{
	struct ssaiot_sc_accel_priv *priv = ssaiot_sc_accel_iio_priv(indio_dev);

	return test_bit(chan->address, &priv->events);
}

static int ssaiot_sc_accel_write_event_config(struct iio_dev *indio_dev,
					      const struct iio_chan_spec *chan,
					      enum iio_event_type type,
					      enum iio_event_direction dir,
					      int state)
{
	struct ssaiot_sc_accel_priv *priv = ssaiot_sc_accel_iio_priv(indio_dev);

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

static const struct iio_info ssaiot_sc_accel_motion_info = {
	.read_raw = ssaiot_sc_accel_motion_read_raw,
	.read_event_config = ssaiot_sc_accel_read_event_config,
	.write_event_config = ssaiot_sc_accel_write_event_config,
};

#define SSAIOT_SC_ACCEL_CHANNEL(_axis) {				\
	.type = IIO_ACCEL,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.scan_index = SSAIOT_SC_ACCEL_##_axis,				\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_CPU,					\
	},								\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
}

/* the detectors are firmware configured, so only reporting can be turned off */
#define SSAIOT_SC_ACCEL_EVENT_SPEC(_type, _dir) {			\
	.type = _type,							\
	.dir = _dir,							\
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),			\
}

static const struct iio_event_spec ssaiot_sc_accel_motion_event_spec[] = {
	SSAIOT_SC_ACCEL_EVENT_SPEC(IIO_EV_TYPE_MAG_ADAPTIVE, IIO_EV_DIR_RISING),
};

static const struct iio_event_spec ssaiot_sc_accel_tilt_event_spec[] = {
	SSAIOT_SC_ACCEL_EVENT_SPEC(IIO_EV_TYPE_CHANGE, IIO_EV_DIR_EITHER),
};

static const struct iio_event_spec ssaiot_sc_accel_freefall_event_spec[] = {
	SSAIOT_SC_ACCEL_EVENT_SPEC(IIO_EV_TYPE_MAG, IIO_EV_DIR_FALLING),
};

/* event only channels, carrying no data of their own */
#define SSAIOT_SC_ACCEL_EVENT_CHANNEL(_type, _mod, _addr, _spec) {	\
	.type = _type,							\
	.modified = _mod != 0,						\
	.channel2 = _mod,						\
	.address = _addr,						\
	.scan_index = -1,						\
	.event_spec = _spec,						\
	.num_event_specs = ARRAY_SIZE(_spec),				\
}

static const struct iio_chan_spec ssaiot_sc_accel_motion_channels[] = {
	SSAIOT_SC_ACCEL_CHANNEL(X),
	SSAIOT_SC_ACCEL_CHANNEL(Y),
	SSAIOT_SC_ACCEL_CHANNEL(Z),
	IIO_CHAN_SOFT_TIMESTAMP(3),
	SSAIOT_SC_ACCEL_EVENT_CHANNEL(IIO_ACCEL, IIO_MOD_X_OR_Y_OR_Z,
				      SSAIOT_SC_ACCEL_EV_MOTION,
				      ssaiot_sc_accel_motion_event_spec),
	SSAIOT_SC_ACCEL_EVENT_CHANNEL(IIO_ACCEL, IIO_MOD_X_AND_Y_AND_Z,
				      SSAIOT_SC_ACCEL_EV_FREEFALL,
				      ssaiot_sc_accel_freefall_event_spec),
	SSAIOT_SC_ACCEL_EVENT_CHANNEL(IIO_INCLI, 0,
				      SSAIOT_SC_ACCEL_EV_TILT,
				      ssaiot_sc_accel_tilt_event_spec),
};

/* always scan all three axes, and let the core demux what the reader enabled */
static const unsigned long ssaiot_sc_accel_motion_scan_masks[] = {
	BIT(SSAIOT_SC_ACCEL_X) | BIT(SSAIOT_SC_ACCEL_Y) | BIT(SSAIOT_SC_ACCEL_Z),
	0
};

static int ssaiot_sc_accel_request_event(struct device *dev,
					 struct ssaiot_sc_accel_priv *priv,
					 enum ssaiot_sc_accel_ev ev,
					 const char *name, irq_handler_t handler)
{
	int irq, ret;

	irq = platform_get_irq_byname(to_platform_device(dev), name);
	if (irq < 0)
		return irq;

	ret = devm_request_threaded_irq(dev, irq, NULL, handler,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					name, priv->motion_iio);
	if (ret)
		return ret;

	priv->event_irq[ev] = irq;

	return 0;
}

static int ssaiot_sc_accel_probe_motion(struct device *dev,
					struct ssaiot_sc_accel_priv *priv)
{
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(priv));
	if (!indio_dev)
		return -ENOMEM;

	*(struct ssaiot_sc_accel_priv **)iio_priv(indio_dev) = priv;
	priv->motion_iio = indio_dev;
	INIT_DELAYED_WORK(&priv->motion_work, ssaiot_sc_accel_motion_poll);

	indio_dev->name = "ssaiot-sc-accel";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ssaiot_sc_accel_motion_info;
	indio_dev->channels = ssaiot_sc_accel_motion_channels;
	indio_dev->num_channels = ARRAY_SIZE(ssaiot_sc_accel_motion_channels);
	indio_dev->available_scan_masks = ssaiot_sc_accel_motion_scan_masks;

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev,
					  &ssaiot_sc_accel_motion_setup_ops);
	if (ret)
		return ret;

	ret = ssaiot_sc_accel_request_event(dev, priv, SSAIOT_SC_ACCEL_EV_MOTION,
					    "motion", ssaiot_sc_accel_motion_event);
	if (ret)
		return ret;

	ret = ssaiot_sc_accel_request_event(dev, priv, SSAIOT_SC_ACCEL_EV_TILT,
					    "tilt", ssaiot_sc_accel_tilt_event);
	if (ret)
		return ret;

	ret = ssaiot_sc_accel_request_event(dev, priv, SSAIOT_SC_ACCEL_EV_FREEFALL,
					    "freefall", ssaiot_sc_accel_freefall_event);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

/*
 * Read captured die temperatures once and push them to the buffer, then
 * re-queue: with no delay while responses come back full, otherwise after the
 * idle interval. Runs only while the buffer is enabled.
 */
static void ssaiot_sc_accel_temp_poll(struct work_struct *work)
{
	struct ssaiot_sc_accel_priv *priv =
		container_of(to_delayed_work(work),
			     struct ssaiot_sc_accel_priv, temp_work);
	unsigned int delay_ms = SSAIOT_SC_ACCEL_TEMP_IDLE_MS;
	struct ssaiot_sc_accel_temp_resp resp;
	s64 ts_ref;
	u8 len;
	int n, i;

	n = ssaiot_sc_xfer(priv->sc, SSAIOT_SC_CMD_SENSOR_READ,
			   SSAIOT_SC_SENSOR_ACCEL_TEMP, NULL, 0,
			   (u8 *)&resp, sizeof(resp), &len, NULL, &ts_ref);
	if (!n)
		n = ssaiot_sc_accel_records(len, offsetof(typeof(resp), sample),
					    sizeof(resp.sample[0]));

	if (n < 0) {
		dev_warn_ratelimited(priv->sc->dev,
				     "temperature read failed: %d.\n", n);
	} else {
		for (i = 0; i < n; i++) {
			priv->temp_scan.temp =
				(s16)le16_to_cpu(resp.sample[i].temp);

			iio_push_to_buffers_with_timestamp(priv->temp_iio,
					&priv->temp_scan,
					ssaiot_sc_accel_timestamp(ts_ref, resp.now,
							resp.sample[i].timestamp));
		}

		/* a full response says the stream held at least this much more */
		if (n == ARRAY_SIZE(resp.sample))
			delay_ms = 0;
	}

	queue_delayed_work(priv->sc->wq, &priv->temp_work,
			   msecs_to_jiffies(delay_ms));
}

static int ssaiot_sc_accel_temp_postenable(struct iio_dev *indio_dev)
{
	struct ssaiot_sc_accel_priv *priv = ssaiot_sc_accel_iio_priv(indio_dev);

	queue_delayed_work(priv->sc->wq, &priv->temp_work, 0);

	return 0;
}

static int ssaiot_sc_accel_temp_predisable(struct iio_dev *indio_dev)
{
	struct ssaiot_sc_accel_priv *priv = ssaiot_sc_accel_iio_priv(indio_dev);

	/* safe against the work re-queueing itself */
	cancel_delayed_work_sync(&priv->temp_work);

	return 0;
}

static const struct iio_buffer_setup_ops ssaiot_sc_accel_temp_setup_ops = {
	.postenable = ssaiot_sc_accel_temp_postenable,
	.predisable = ssaiot_sc_accel_temp_predisable,
};

/* no IIO_CHAN_INFO_RAW here either, for the same reason as the axes */
static int ssaiot_sc_accel_temp_read_raw(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		/*
		 * 256 LSB per degree against an abi in millidegrees, so
		 * 1000 / 256 per LSB, applied after the offset.
		 */
		*val = 3;
		*val2 = 906250;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		/* zero means 25 degrees, which is 25 * 256 LSB from zero */
		*val = 6400;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static const struct iio_info ssaiot_sc_accel_temp_info = {
	.read_raw = ssaiot_sc_accel_temp_read_raw,
};

static const struct iio_chan_spec ssaiot_sc_accel_temp_channels[] = {
	{
		/* sensor die temperature */
		.type = IIO_TEMP,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static int ssaiot_sc_accel_probe_temp(struct device *dev,
				      struct ssaiot_sc_accel_priv *priv)
{
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(priv));
	if (!indio_dev)
		return -ENOMEM;

	*(struct ssaiot_sc_accel_priv **)iio_priv(indio_dev) = priv;
	priv->temp_iio = indio_dev;
	INIT_DELAYED_WORK(&priv->temp_work, ssaiot_sc_accel_temp_poll);

	indio_dev->name = "ssaiot-sc-accel-temp";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ssaiot_sc_accel_temp_info;
	indio_dev->channels = ssaiot_sc_accel_temp_channels;
	indio_dev->num_channels = ARRAY_SIZE(ssaiot_sc_accel_temp_channels);

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev,
					  &ssaiot_sc_accel_temp_setup_ops);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static int ssaiot_sc_accel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ssaiot_sc_accel_priv *priv;
	int ret;

	/* the mfd cell has no dedicated dt node, reuse parent */
	dev->of_node = dev->parent->of_node;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->sc = dev_get_drvdata(dev->parent);

	device_init_wakeup(dev, true);

	ret = ssaiot_sc_irq_claim(priv->sc, SSAIOT_SC_INT_SRC_ACC,
				  device_may_wakeup(dev));
	if (ret)
		return ret;

	ret = ssaiot_sc_accel_probe_motion(dev, priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register motion device.\n");

	ret = ssaiot_sc_accel_probe_temp(dev, priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register temperature device.\n");

	return 0;
}

/* prepare for shutdown, i.e. release the bus and apply the wake-up policy */
static void ssaiot_sc_accel_shutdown(struct platform_device *pdev)
{
	struct ssaiot_sc_accel_priv *priv = platform_get_drvdata(pdev);

	/* stop work to release the bus */
	cancel_delayed_work_sync(&priv->motion_work);
	cancel_delayed_work_sync(&priv->temp_work);

	/* apply wake-up policy */
	ssaiot_sc_irq_set_poweron(priv->sc, SSAIOT_SC_INT_SRC_ACC,
				  device_may_wakeup(&pdev->dev));
}

/*
 * Only the detectors userspace enabled can assert, so those are the ones to
 * arm. Their wake references have to be given back one for one, which holds
 * because userspace is frozen before this runs and cannot change the set.
 */
static int ssaiot_sc_accel_suspend(struct device *dev)
{
	struct ssaiot_sc_accel_priv *priv = dev_get_drvdata(dev);
	unsigned int ev, armed;
	int ret;

	/* check if device is set as wakeup source */
	if (!device_may_wakeup(dev))
		return 0;

	/* enable irq wakeup */
	for_each_set_bit(ev, &priv->events, SSAIOT_SC_ACCEL_EV_MAX) {
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

static int ssaiot_sc_accel_resume(struct device *dev)
{
	struct ssaiot_sc_accel_priv *priv = dev_get_drvdata(dev);
	unsigned int ev;

	/* check if device was set as wakeup source */
	if (!device_may_wakeup(dev))
		return 0;

	/* disable irq wakeup */
	for_each_set_bit(ev, &priv->events, SSAIOT_SC_ACCEL_EV_MAX)
		disable_irq_wake(priv->event_irq[ev]);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ssaiot_sc_accel_pm_ops, ssaiot_sc_accel_suspend,
				ssaiot_sc_accel_resume);

/*
 * The id must match the MFD cell name and is capped at PLATFORM_NAME_SIZE,
 * so it stays short. Since an id table suppresses the driver name fallback in
 * platform_match(), the driver name itself is free to be descriptive.
 */
static const struct platform_device_id ssaiot_sc_accel_id_table[] = {
	{ "ssaiot-sc-acc", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, ssaiot_sc_accel_id_table);

static struct platform_driver ssaiot_sc_accel_driver = {
	.driver = {
		.name = "solidsense-aiot-system-controller-accel",
		.pm = pm_sleep_ptr(&ssaiot_sc_accel_pm_ops),
	},
	.probe = ssaiot_sc_accel_probe,
	.shutdown = ssaiot_sc_accel_shutdown,
	.id_table = ssaiot_sc_accel_id_table,
};
module_platform_driver(ssaiot_sc_accel_driver);

MODULE_AUTHOR("Josua Mayer");
MODULE_DESCRIPTION("SolidRun SolidSense AIOT Board System Controller Accelerometer Driver");
MODULE_LICENSE("GPL v2");
