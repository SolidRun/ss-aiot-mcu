// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */
#ifndef _SSAIOT_SC_H_
#define _SSAIOT_SC_H_

#include <linux/limits.h>
#include <linux/mutex.h>

/* transport protocol definitions */
#define SSAIOT_SC_CMD_HDR_LEN		3	/* CMD + SENSOR_ID + DATA_LEN */
#define SSAIOT_SC_RESP_HDR_LEN		2	/* STATUS + DATA_LEN */
/* largest command payload the controller accepts */
#define SSAIOT_SC_CMD_MAX_DATA_LEN	U8_MAX
/* largest response payload, reading past it stretches SCL and wedges the bus */
#define SSAIOT_SC_RESP_MAX_DATA_LEN	U8_MAX

#define SSAIOT_SC_STATUS_OK		0x00
#define SSAIOT_SC_STATUS_ERROR		0x01

/* application protocol definitions */
#define SSAIOT_SC_CMD_SENSOR_ON		0x10
#define SSAIOT_SC_CMD_SENSOR_OFF	0x11
#define SSAIOT_SC_CMD_SENSOR_READ	0x12
#define SSAIOT_SC_CMD_SENSOR_CONFIG	0x13

#define SSAIOT_SC_SENSOR_LED		0x01
#define SSAIOT_SC_SENSOR_IR		0x02
#define SSAIOT_SC_SENSOR_ACCEL_MOTION	0x03
#define SSAIOT_SC_SENSOR_GPS		0x04
#define SSAIOT_SC_SENSOR_CHARGER	0x05
#define SSAIOT_SC_SENSOR_RTC		0x06
#define SSAIOT_SC_SENSOR_INTERRUPTS	0x07
#define SSAIOT_SC_SENSOR_ALARM		0x08
#define SSAIOT_SC_SENSOR_SOM		0x09
#define SSAIOT_SC_SENSOR_ACCEL_TEMP	0x0a

/* controller interrupt sources */
enum ssaiot_sc_int_src {
	SSAIOT_SC_INT_SRC_MCU = 0,
	SSAIOT_SC_INT_SRC_IR,
	SSAIOT_SC_INT_SRC_ACC,
	SSAIOT_SC_INT_SRC_RTC,
	SSAIOT_SC_INT_SRC_MAX,
};

/* payload of CMD_SENSOR_CONFIG / SENSOR_INTERRUPTS, as the controller lays it out */
struct ssaiot_sc_irq_config {
	u8 en_sources; /* sources that may be reported at all */
	u8 pwr_sources; /* sources that may also power the SoM on */
	u8 en_detail[SSAIOT_SC_INT_SRC_MAX]; /* per source, indexed by it */
} __packed;

struct ssaiot_sc_priv {
	struct device *dev;
	int irq;
	struct irq_domain *irq_domain;

	/*
	 * Interrupt configuration as the controller lays it out, and the state
	 * needed to keep it there. All of it belongs to irq.c.
	 */
	struct mutex irq_lock;
	struct ssaiot_sc_irq_config irq_config;
	bool irq_config_dirty;

	/*
	 * Shared ordered queue, free for the core and any sub-device to use.
	 * Every command occupies the I2C bus for the duration of a transfer, so
	 * deferred work serialises against itself regardless; running one item
	 * at a time here makes that explicit and costs one worker rather than a
	 * task per user. Work that re-queues itself yields to whatever else is
	 * already pending, so a busy user cannot starve the others.
	 */
	struct workqueue_struct *wq;
};

/* transport api (transport.c) */

int ssaiot_sc_xfer(struct ssaiot_sc_priv *priv, u8 cmd, u8 sensor_id,
		   const u8 *tx, u8 tx_len, u8 *rx, u8 rx_size,
		   u8 *rx_data_len, u8 *status, s64 *ts);

/* irq api (irq.c) */

/* translated virqs, the first of them handled by the core itself */
enum ssaiot_sc_irq {
	SSAIOT_SC_IRQ_MCU_RESTART = 0,
	SSAIOT_SC_IRQ_IR_ACTIVITY,
	SSAIOT_SC_IRQ_IR_PRESENCE,
	SSAIOT_SC_IRQ_ACCEL_MOTION,
	SSAIOT_SC_IRQ_ACCEL_TILT,
	SSAIOT_SC_IRQ_ACCEL_FREEFALL,
	SSAIOT_SC_IRQ_RTC_ALARM,
	SSAIOT_SC_NUM_IRQS,
};

int ssaiot_sc_irq_probe(struct device *dev);
void ssaiot_sc_irq_shutdown(struct ssaiot_sc_priv *priv);
int ssaiot_sc_irq_claim(struct ssaiot_sc_priv *priv,
			enum ssaiot_sc_int_src src, bool poweron);
int ssaiot_sc_irq_set_poweron(struct ssaiot_sc_priv *priv,
			      enum ssaiot_sc_int_src src, bool on);

/* mfd api (mfd.c) */

int ssaiot_sc_mfd_probe(struct device *dev);

#endif /* _SSAIOT_SC_H_ */
