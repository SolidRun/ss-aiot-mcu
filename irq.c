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
 * Byte offsets within the payload of CMD_SENSOR_READ / SENSOR_INTERRUPTS.
 * All three sources are cleared by that read, so it is the only place the
 * interrupt line may be sampled - a sub-device polling its own sensor would
 * consume events belonging to the others.
 */
#define SSAIOT_SC_INT_MCU		0
#define SSAIOT_SC_INT_IR		1
#define SSAIOT_SC_INT_ACC		2
#define SSAIOT_SC_INT_LEN		3

/* Codes reported in the IR and ACC bytes */
#define SSAIOT_SC_IR_MOTION		BIT(1)	/* 0x02 */
#define SSAIOT_SC_IR_PRESENCE		BIT(2)	/* 0x04 */
#define SSAIOT_SC_ACC_MOTION		BIT(0)	/* 0x01 */

/* Demultiplexing table: which payload byte and which bit raises each IRQ */
static const struct {
	u8 offset;
	u8 mask;
} ssaiot_sc_irq_source[SSAIOT_SC_NUM_IRQS] = {
	[SSAIOT_SC_IRQ_IR_MOTION]   = { SSAIOT_SC_INT_IR,  SSAIOT_SC_IR_MOTION },
	[SSAIOT_SC_IRQ_IR_PRESENCE] = { SSAIOT_SC_INT_IR,  SSAIOT_SC_IR_PRESENCE },
	[SSAIOT_SC_IRQ_ACC_MOTION]  = { SSAIOT_SC_INT_ACC, SSAIOT_SC_ACC_MOTION },
};

/*
 * The controller has no interrupt mask registers - a source is either reported
 * by the interrupt read or it is not - so the chip implements no callbacks at
 * all. mask_irq() and unmask_irq() skip an absent handler, and disable_irq()
 * still takes effect because it sets IRQD_IRQ_DISABLED itself, which is what
 * handle_nested_irq() tests before running the action.
 */
static struct irq_chip ssaiot_sc_irq_chip = {
	.name = "ssaiot-sc",
};

static int ssaiot_sc_irq_map(struct irq_domain *d, unsigned int virq,
			     irq_hw_number_t hwirq)
{
	struct ssaiot_sc_priv *priv = d->host_data;

	/*
	 * handle_nested_irq() invokes the action directly, so there is no flow
	 * handler, and the chip declares no callbacks that could read chip data.
	 *
	 * The parent has to be recorded: irq_sw_resend() refuses to replay a
	 * pending interrupt on a nested thread whose desc has no parent_irq, so
	 * without it an event latched while a sub-device had its IRQ disabled
	 * would be dropped instead of resent on enable_irq().
	 */
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
 * necessary and sufficient. The first byte of that status says whether the
 * controller is asserting at all, which is what decides whether the interrupt
 * was ours to claim.
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
	if (!flags[SSAIOT_SC_INT_MCU])
		return IRQ_NONE;

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
