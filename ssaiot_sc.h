// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */
#ifndef _SSAIOT_SC_H_
#define _SSAIOT_SC_H_

//#include <linux/i2c.h>
//#include <linux/types.h>

/* transport protocol definitions */
#define SSAIOT_SC_CMD_HDR_LEN		3	/* CMD + SENSOR_ID + DATA_LEN */
#define SSAIOT_SC_RESP_HDR_LEN		2	/* STATUS + DATA_LEN */
#define SSAIOT_SC_MAX_DATA_LEN		32	/* payload cap of both directions */

#define SSAIOT_SC_STATUS_OK		0x00
#define SSAIOT_SC_STATUS_ERROR		0x01

/* application protocol definitions */
#define SSAIOT_SC_CMD_SENSOR_ON		0x10
#define SSAIOT_SC_CMD_SENSOR_OFF	0x11
#define SSAIOT_SC_CMD_SENSOR_READ	0x12
#define SSAIOT_SC_CMD_SENSOR_CONFIG	0x13

#define SSAIOT_SC_SENSOR_LED		0x01
#define SSAIOT_SC_SENSOR_IR		0x02
#define SSAIOT_SC_SENSOR_ACC		0x03
#define SSAIOT_SC_SENSOR_GPS		0x04
#define SSAIOT_SC_SENSOR_CHARGER	0x05
#define SSAIOT_SC_SENSOR_RTC		0x06
#define SSAIOT_SC_SENSOR_INTERRUPTS	0x07
#define SSAIOT_SC_SENSOR_ALARM		0x08
#define SSAIOT_SC_SENSOR_SOM		0x09

struct ssaiot_sc_priv {
	struct device *dev;
	int irq;
	struct irq_domain *irq_domain;

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
		   const u8 *tx, u8 tx_len, u8 *rx, u8 rx_len, u8 *status);

/* irq api (irq.c) */

enum ssaiot_sc_irq {
	SSAIOT_SC_IRQ_IR_MOTION = 0,
	SSAIOT_SC_IRQ_IR_PRESENCE,
	SSAIOT_SC_IRQ_ACC_WAKEUP,
	SSAIOT_SC_IRQ_RTC_ALARM,
	SSAIOT_SC_NUM_IRQS,
};

int ssaiot_sc_irq_probe(struct device *dev);

/* mfd api (mfd.c) */

int ssaiot_sc_mfd_probe(struct device *dev);

#endif /* _SSAIOT_SC_H_ */
