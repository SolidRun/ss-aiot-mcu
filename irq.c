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
#include <linux/string.h>

#include "ssaiot_sc.h"

/*
 * Payload of CMD_SENSOR_READ / SENSOR_INTERRUPTS: a bitfield of the sources
 * that fired, then one detail byte per source, indexed by the source itself -
 * the same shape the configuration payload uses for its enable masks.
 *
 * Every byte is an accumulated latch that the read clears, so this is the only
 * place the interrupt state may be sampled - a sub-device reading it for itself
 * would consume events belonging to the others. It answers "what fired since
 * the last read", never "what is true now".
 */
struct ssaiot_sc_irq_status {
	u8 sources;
	u8 detail[SSAIOT_SC_INT_SRC_MAX];
} __packed;

/* mcu detail */
#define SSAIOT_SC_MCU_RESTART		BIT(0)

/* ir detail */
#define SSAIOT_SC_IR_MOTION		BIT(0)
#define SSAIOT_SC_IR_PRESENCE		BIT(1)

/* accelerometer detail */
#define SSAIOT_SC_ACCEL_MOTION		BIT(0)
#define SSAIOT_SC_ACCEL_TILT		BIT(1)
#define SSAIOT_SC_ACCEL_FREEFALL	BIT(2)

/* rtc detail */
#define SSAIOT_SC_RTC_ALARM_A		BIT(0)

/* Demultiplexing table: which source and which of its detail bits raises each IRQ. */
static const struct {
	enum ssaiot_sc_int_src src;
	u8 mask;
} ssaiot_sc_irq_source[SSAIOT_SC_NUM_IRQS] = {
	[SSAIOT_SC_IRQ_MCU_RESTART]    = { SSAIOT_SC_INT_SRC_MCU, SSAIOT_SC_MCU_RESTART },
	[SSAIOT_SC_IRQ_IR_ACTIVITY]    = { SSAIOT_SC_INT_SRC_IR,  SSAIOT_SC_IR_MOTION },
	[SSAIOT_SC_IRQ_IR_PRESENCE]    = { SSAIOT_SC_INT_SRC_IR,  SSAIOT_SC_IR_PRESENCE },
	[SSAIOT_SC_IRQ_ACCEL_MOTION]   = { SSAIOT_SC_INT_SRC_ACC, SSAIOT_SC_ACCEL_MOTION },
	[SSAIOT_SC_IRQ_ACCEL_TILT]     = { SSAIOT_SC_INT_SRC_ACC, SSAIOT_SC_ACCEL_TILT },
	[SSAIOT_SC_IRQ_ACCEL_FREEFALL] = { SSAIOT_SC_INT_SRC_ACC, SSAIOT_SC_ACCEL_FREEFALL },
	[SSAIOT_SC_IRQ_RTC_ALARM]      = { SSAIOT_SC_INT_SRC_RTC, SSAIOT_SC_RTC_ALARM_A },
};

/* sync shadow state with controller */
static int ssaiot_sc_irq_sync(struct ssaiot_sc_priv *priv)
{
	struct ssaiot_sc_irq_config resp;
	int ret;

	ret = ssaiot_sc_xfer(priv, SSAIOT_SC_CMD_SENSOR_CONFIG,
			     SSAIOT_SC_SENSOR_INTERRUPTS,
			     (const u8 *)&priv->irq_config,
			     sizeof(priv->irq_config),
			     (u8 *)&resp, sizeof(resp), NULL, NULL, NULL);
	if (ret) {
		dev_err_ratelimited(priv->dev,
				    "failed to write interrupt config: %d.\n",
				    ret);
		return ret;
	}

	if (memcmp(&resp, &priv->irq_config, sizeof(resp)))
		dev_warn(priv->dev,
			 "interrupt config not applied: wrote %*ph, got %*ph.\n",
			 (int)sizeof(priv->irq_config), &priv->irq_config,
			 (int)sizeof(resp), &resp);

	priv->irq_config_dirty = false;

	return 0;
}

static void ssaiot_sc_irq_set_enable(struct irq_data *d, bool on)
{
	struct ssaiot_sc_priv *priv = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	enum ssaiot_sc_int_src src = ssaiot_sc_irq_source[hwirq].src;
	u8 *detail = &priv->irq_config.en_detail[src];
	u8 before = *detail;

	if (on)
		*detail |= ssaiot_sc_irq_source[hwirq].mask;
	else
		*detail &= ~ssaiot_sc_irq_source[hwirq].mask;

	priv->irq_config_dirty |= *detail != before;
}

static void ssaiot_sc_irq_mask(struct irq_data *d)
{
	ssaiot_sc_irq_set_enable(d, false);
}

static void ssaiot_sc_irq_unmask(struct irq_data *d)
{
	ssaiot_sc_irq_set_enable(d, true);
}

/* mask an interrupt when its first handler is installed */
static int ssaiot_sc_irq_request_resources(struct irq_data *d)
{
	ssaiot_sc_irq_set_enable(d, false);

	return 0;
}

static void ssaiot_sc_irq_bus_lock(struct irq_data *d)
{
	struct ssaiot_sc_priv *priv = irq_data_get_irq_chip_data(d);

	mutex_lock(&priv->irq_lock);
}

static void ssaiot_sc_irq_bus_sync_unlock(struct irq_data *d)
{
	struct ssaiot_sc_priv *priv = irq_data_get_irq_chip_data(d);

	if (priv->irq_config_dirty)
		ssaiot_sc_irq_sync(priv);

	mutex_unlock(&priv->irq_lock);
}

/* forward set_wake to the parent */
static int ssaiot_sc_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct ssaiot_sc_priv *priv = irq_data_get_irq_chip_data(d);

	return irq_set_irq_wake(priv->irq, on);
}

/*
 * mask and unmask only edit the shadow, because the core runs them under the
 * descriptor's raw spinlock where the bus may not be used. It brackets them
 * with bus_lock and bus_sync_unlock, which run outside that lock, so the one
 * transfer happens there however many bits changed. request_resources runs
 * inside the same bracket and edits the shadow for the same reason, so taking
 * an interrupt over and unmasking it cost one transfer between them.
 *
 * disable has to be given too. Without it disable_irq() is lazy: it records
 * IRQD_IRQ_DISABLED and leaves the masking to the flow handler, and
 * handle_nested_irq() only marks such an interrupt pending. The controller
 * would go on reporting a detector userspace had switched off.
 *
 * Wake-up is a different thing and stays separate: it keeps the parent's line
 * alive across suspend and tells the controller nothing, whose power-on mask
 * matters only once the SoM has no power at all.
 */
static struct irq_chip ssaiot_sc_irq_chip = {
	.name = "ssaiot-sc",
	.irq_mask = ssaiot_sc_irq_mask,
	.irq_disable = ssaiot_sc_irq_mask,
	.irq_unmask = ssaiot_sc_irq_unmask,
	.irq_request_resources = ssaiot_sc_irq_request_resources,
	.irq_bus_lock = ssaiot_sc_irq_bus_lock,
	.irq_bus_sync_unlock = ssaiot_sc_irq_bus_sync_unlock,
	.irq_set_wake = ssaiot_sc_irq_set_wake,
};

/**
 * ssaiot_sc_irq_set_poweron() - Let a source restore SoM power
 * @priv: Driver private structure
 * @src: Source to allow or deny
 * @on: Whether that source may power the SoM on
 *
 * The controller consults this only while the SoM is off, so a sub-device sets
 * it as it probes and again on the way down, rather than keeping it current -
 * nothing reports a write to power/wakeup while running. Granularity is the
 * individual interrupt, which is all that attribute can express anyway, since
 * every cell owns exactly one source.
 *
 * Return: 0 on success, negative errno on failure.
 */
int ssaiot_sc_irq_set_poweron(struct ssaiot_sc_priv *priv,
			      enum ssaiot_sc_int_src src, bool on)
{
	int ret;

	mutex_lock(&priv->irq_lock);
	if (on)
		priv->irq_config.pwr_sources |= BIT(src);
	else
		priv->irq_config.pwr_sources &= ~BIT(src);
	ret = ssaiot_sc_irq_sync(priv);
	mutex_unlock(&priv->irq_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ssaiot_sc_irq_set_poweron);

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

/*
 * One cell, carrying the number of the interrupt alone. The controller decides
 * for itself what raises each one, so there is no trigger type for a consumer
 * to ask for.
 */
static const struct irq_domain_ops ssaiot_sc_irq_domain_ops = {
	.map = ssaiot_sc_irq_map,
	.xlate = irq_domain_xlate_onecell,
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
	struct ssaiot_sc_irq_status status;
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
			     (u8 *)&status, sizeof(status), NULL, NULL, NULL);
	if (ret) {
		/* delay next attempt in case of bus errors */
		msleep(10);
		return IRQ_NONE;
	}

	/* somebody else pulled the line down, leave it to them */
	if (!status.sources)
		return IRQ_NONE;

	for (i = 0; i < SSAIOT_SC_NUM_IRQS; i++) {
		if (!(status.detail[ssaiot_sc_irq_source[i].src] &
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

/**
 * ssaiot_sc_irq_restart() - Answer the controller having restarted
 * @irq: Restart interrupt number
 * @data: Driver private structure
 *
 * The configuration went with it, back to reporting everything. Nothing
 * reaches userspace, since a masked source still has its virq disabled, but
 * the controller asserts and is read for events nobody wants.
 */
static irqreturn_t ssaiot_sc_irq_restart(int irq, void *data)
{
	struct ssaiot_sc_priv *priv = data;

	dev_warn(priv->dev,
		 "controller restarted, its configuration is back at defaults.\n");

	/*
	 * Marked dirty first, so a write that fails here is retried by whatever
	 * changes a mask next - the restart is announced once and does not come
	 * round again.
	 */
	mutex_lock(&priv->irq_lock);
	priv->irq_config_dirty = true;
	ssaiot_sc_irq_sync(priv);
	mutex_unlock(&priv->irq_lock);

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
	unsigned int virq;
	int ret;

	/*
	 * Bound to the controller's own node, which declares itself an
	 * interrupt controller, so that a sub-device names the interrupt it
	 * wants in the device tree and of_irq_get() finds this domain.
	 */
	priv->irq_domain = irq_domain_add_linear(dev_of_node(dev),
						 SSAIOT_SC_NUM_IRQS,
						 &ssaiot_sc_irq_domain_ops,
						 priv);
	if (!priv->irq_domain)
		return dev_err_probe(dev, -ENOMEM,
				     "Failed to add irq domain.\n");

	ret = devm_add_action_or_reset(dev, ssaiot_sc_irq_domain_release,
				       priv->irq_domain);
	if (ret)
		return ret;

	mutex_init(&priv->irq_lock);

	/*
	 * Seed shadow state from the active controller configuration rather
	 * than narrow it here. Each source is taken over by its own driver as
	 * that probes; one no driver claims keeps the state it booted with.
	 */
	ret = ssaiot_sc_xfer(priv, SSAIOT_SC_CMD_SENSOR_CONFIG,
			     SSAIOT_SC_SENSOR_INTERRUPTS, NULL, 0,
			     (u8 *)&priv->irq_config, sizeof(priv->irq_config),
			     NULL, NULL, NULL);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to read interrupt config.\n");

	/*
	 * A sub-device's mapping is created when it resolves its own interrupt
	 * against this domain. The controller's own restart is named by nobody,
	 * so it is mapped here.
	 */
	virq = irq_create_mapping(priv->irq_domain, SSAIOT_SC_IRQ_MCU_RESTART);
	if (!virq)
		return dev_err_probe(dev, -ENOMEM,
				     "Failed to map restart irq.\n");

	ret = devm_request_threaded_irq(dev, virq, NULL, ssaiot_sc_irq_restart,
					IRQF_ONESHOT, dev_name(dev), priv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request restart irq.\n");

	ret = devm_request_threaded_irq(dev, priv->irq, NULL,
					ssaiot_sc_irq_thread, IRQF_ONESHOT,
					dev_name(dev), priv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request irq.\n");

	return 0;
}

/* prepare for shutdown, i.e. disable interrupts */
void ssaiot_sc_irq_shutdown(struct ssaiot_sc_priv *priv)
{
	unsigned int virq = irq_find_mapping(priv->irq_domain, SSAIOT_SC_IRQ_MCU_RESTART);

	if (virq)
		disable_irq(virq);
}
