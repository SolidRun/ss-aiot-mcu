// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>

#include "ssaiot_sc.h"

/*
 * Payload of CMD_SENSOR_READ / SENSOR_INTERRUPTS: a bitfield of the sources
 * that fired, followed by one detail byte for each source that has one.
 *
 * Every byte is an accumulated latch that the read clears, so this is the only
 * place the interrupt state may be sampled - a sub-device reading it for itself
 * would consume events belonging to the others. It answers "what fired since
 * the last read", never "what is true now".
 */
#define SSAIOT_SC_INT_SOURCES		0
#define SSAIOT_SC_INT_IR		1
#define SSAIOT_SC_INT_ACC		2
#define SSAIOT_SC_INT_RTC		3
#define SSAIOT_SC_INT_LEN		4

/* data[0], which sources fired */
#define SSAIOT_SC_INT_SRC_MCU		BIT(0)
#define SSAIOT_SC_INT_SRC_IR		BIT(1)
#define SSAIOT_SC_INT_SRC_ACC		BIT(2)
#define SSAIOT_SC_INT_SRC_RTC		BIT(3)
#define SSAIOT_SC_INT_SRC_CHARGER	BIT(4)	/* allocated, never set yet */

/* data[1], masked FUNC_STATUS of the IR sensor */
#define SSAIOT_SC_IR_MOTION		BIT(1)
#define SSAIOT_SC_IR_PRESENCE		BIT(2)

/* data[2], WAKE_UP_SRC of the accelerometer, the axis bits left aside */
#define SSAIOT_SC_ACC_WAKEUP		BIT(3)
#define SSAIOT_SC_ACC_FREEFALL		BIT(5)
#define SSAIOT_SC_ACC_SLEEP_CHANGE	BIT(6)

/* data[3] */
#define SSAIOT_SC_RTC_ALARM_A		BIT(0)

/*
 * Demultiplexing table: which payload byte and which bit raises each IRQ. A
 * source with no detail byte is dispatched from its own bit in data[0].
 */
static const struct {
	u8 offset;
	u8 mask;
} ssaiot_sc_irq_source[SSAIOT_SC_NUM_IRQS] = {
	[SSAIOT_SC_IRQ_IR_MOTION]   = { SSAIOT_SC_INT_IR,  SSAIOT_SC_IR_MOTION },
	[SSAIOT_SC_IRQ_IR_PRESENCE] = { SSAIOT_SC_INT_IR,  SSAIOT_SC_IR_PRESENCE },
	[SSAIOT_SC_IRQ_ACC_WAKEUP]  = { SSAIOT_SC_INT_ACC, SSAIOT_SC_ACC_WAKEUP },
	[SSAIOT_SC_IRQ_RTC_ALARM]   = { SSAIOT_SC_INT_RTC, SSAIOT_SC_RTC_ALARM_A },
};

/* forward set_wake to the parent */
static int ssaiot_sc_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct ssaiot_sc_priv *priv = irq_data_get_irq_chip_data(d);

	return irq_set_irq_wake(priv->irq, on);
}

/*
 * The controller has no interrupt mask registers - a source is either reported
 * by the interrupt read or it is not - so the chip implements no mask
 * callbacks. mask_irq() and unmask_irq() skip an absent handler, and
 * disable_irq() still takes effect because it sets IRQD_IRQ_DISABLED itself,
 * which is what handle_nested_irq() tests before running the action.
 *
 * Wake-up is the one thing the chip has to act on, since the hardware that
 * carries it belongs to the parent rather than to any single source.
 */
static struct irq_chip ssaiot_sc_irq_chip = {
	.name = "ssaiot-sc",
	.irq_set_wake = ssaiot_sc_irq_set_wake,
};

static int ssaiot_sc_irq_map(struct irq_domain *d, unsigned int virq,
			     irq_hw_number_t hwirq)
{
	struct ssaiot_sc_priv *priv = d->host_data;

	/*
	 * handle_nested_irq() invokes the action directly, so there is no flow
	 * handler. The chip data is what set_wake reads to find the parent.
	 *
	 * The parent has to be recorded: irq_sw_resend() refuses to replay a
	 * pending interrupt on a nested thread whose desc has no parent_irq, so
	 * without it an event latched while a sub-device had its IRQ disabled
	 * would be dropped instead of resent on enable_irq().
	 */
	irq_set_chip_data(virq, priv);
	irq_set_chip(virq, &ssaiot_sc_irq_chip);
	irq_set_nested_thread(virq, 1);
	irq_set_parent(virq, priv->irq);
	irq_set_noprobe(virq);

	return 0;
}

static const struct irq_domain_ops ssaiot_sc_irq_domain_ops = {
	.map = ssaiot_sc_irq_map,
};

/**
 * ssaiot_sc_irq_thread() - Demultiplex the controller interrupt
 * @irq: Parent interrupt number
 * @data: Driver private structure
 *
 * Reading the interrupt status both reports and clears every source, and also
 * deasserts the controller's interrupt line, so one read per assertion is both
 * necessary and sufficient. The source bitfield says whether anything on the
 * controller fired at all, which is what decides whether the interrupt was ours
 * to claim.
 */
static irqreturn_t ssaiot_sc_irq_thread(int irq, void *data)
{
	struct ssaiot_sc_priv *priv = data;
	u8 status;
	u8 flags[SSAIOT_SC_INT_LEN];
	unsigned int i, virq;
	int ret;

	/*
	 * On failure the controller has not been read and keeps the line
	 * asserted, so the level interrupt fires again immediately. Report the
	 * interrupt as unhandled and let the core's spurious detection retire
	 * it rather than spin here forever.
	 */
	ret = ssaiot_sc_xfer(priv, SSAIOT_SC_CMD_SENSOR_READ,
			     SSAIOT_SC_SENSOR_INTERRUPTS, NULL, 0,
			     flags, sizeof(flags), &status);
	if (ret) {
		/* delay next attempt in case of bus errors */
		msleep(10);
		return IRQ_NONE;
	}

	/* somebody else pulled the line down, leave it to them */
	if (!flags[SSAIOT_SC_INT_SOURCES])
		return IRQ_NONE;

	/* mcu source means system controller restarted */
	if (flags[SSAIOT_SC_INT_SOURCES] & SSAIOT_SC_INT_SRC_MCU)
		dev_warn(priv->dev,
			 "controller restarted, sensor configuration was lost.\n");

	for (i = 0; i < SSAIOT_SC_NUM_IRQS; i++) {
		if (!(flags[ssaiot_sc_irq_source[i].offset] &
		      ssaiot_sc_irq_source[i].mask))
			continue;

		virq = irq_find_mapping(priv->irq_domain, i);
		if (virq)
			handle_nested_irq(virq);
	}

	/*
	 * The controller was asserting and the read has released it, so the
	 * interrupt is handled even if no individual source claimed it - a
	 * racing sensor read can consume the flag before we get to it.
	 */
	return IRQ_HANDLED;
}

static void ssaiot_sc_irq_domain_release(void *data)
{
	struct irq_domain *domain = data;
	unsigned int i;

	for (i = 0; i < SSAIOT_SC_NUM_IRQS; i++)
		irq_dispose_mapping(irq_find_mapping(domain, i));

	irq_domain_remove(domain);
}

/**
 * ssaiot_sc_irq_probe() - Set up the interrupt demultiplexer
 * @dev: System controller device
 *
 * Must run before the sub-devices are registered, so that the domain exists
 * for mfd_add_devices() to translate their IRQ resources against.
 */
int ssaiot_sc_irq_probe(struct device *dev)
{
	struct ssaiot_sc_priv *priv = dev_get_drvdata(dev);
	int ret;

	priv->irq_domain = irq_domain_add_linear(NULL, SSAIOT_SC_NUM_IRQS,
						 &ssaiot_sc_irq_domain_ops,
						 priv);
	if (!priv->irq_domain)
		return dev_err_probe(dev, -ENOMEM,
				     "Failed to add irq domain.\n");

	ret = devm_add_action_or_reset(dev, ssaiot_sc_irq_domain_release,
				       priv->irq_domain);
	if (ret)
		return ret;

	/*
	 * The mappings themselves are created by mfd_add_devices(), which calls
	 * irq_create_mapping() for every IRQ resource its cells declare and
	 * stores the result in the resource. Nothing needs to be mapped here.
	 */
	ret = devm_request_threaded_irq(dev, priv->irq, NULL,
					ssaiot_sc_irq_thread, IRQF_ONESHOT,
					dev_name(dev), priv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request irq.\n");

	return 0;
}
