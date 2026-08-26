// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

//#include <linux/device.h>
//#include <linux/ioport.h>
#include <linux/mfd/core.h>

#include "ssaiot_sc.h"

static const struct resource ssaiot_sc_ir_resources[] = {
	DEFINE_RES_IRQ_NAMED(SSAIOT_SC_IRQ_IR_MOTION, "motion"),
	DEFINE_RES_IRQ_NAMED(SSAIOT_SC_IRQ_IR_PRESENCE, "presence"),
};

static const struct resource ssaiot_sc_acc_resources[] = {
	DEFINE_RES_IRQ_NAMED(SSAIOT_SC_IRQ_ACC_MOTION, "motion"),
};

/*
 * One cell per logical function of the controller. Sub-devices reach the
 * transport with dev_get_drvdata(pdev->dev.parent) and address their own
 * function through the matching SSAIOT_SC_SENSOR_* id.
 */
static const struct mfd_cell ssaiot_sc_cells[] = {
	{
		.name = "ssaiot-sc-led",
	}, {
		.name = "ssaiot-sc-ir",
		.resources = ssaiot_sc_ir_resources,
		.num_resources = ARRAY_SIZE(ssaiot_sc_ir_resources),
	}, {
		.name = "ssaiot-sc-acc",
		.resources = ssaiot_sc_acc_resources,
		.num_resources = ARRAY_SIZE(ssaiot_sc_acc_resources),
	}, {
		.name = "ssaiot-sc-gnss",
	}, {
		.name = "ssaiot-sc-charger",
	}, {
		.name = "ssaiot-sc-rtc",
	},
};

/**
 * ssaiot_sc_mfd_probe() - Register the sub-devices
 * @dev: System controller device
 *
 * Requires ssaiot_sc_irq_probe() to have run, so that the IRQ resources above
 * can be translated against the controller's interrupt domain.
 */
int ssaiot_sc_mfd_probe(struct device *dev)
{
	struct ssaiot_sc_priv *priv = dev_get_drvdata(dev);
	int ret;

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, ssaiot_sc_cells,
				   ARRAY_SIZE(ssaiot_sc_cells), NULL, 0,
				   priv->irq_domain);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add MFD child devices.\n");

	return 0;
}
