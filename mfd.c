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

/*
 * One cell per logical function of the controller. Sub-devices reach the
 * transport with dev_get_drvdata(pdev->dev.parent) and address their own
 * function through the matching SSAIOT_SC_SENSOR_* id.
 *
 * Each cell names the compatible of its own child node, which mfd_add_device()
 * finds below the parent and hands to the platform device. Interrupts are
 * resolved in DT by phandle and not listed as resources.
 */
static const struct mfd_cell ssaiot_sc_cells[] = {
	{
		.name = "ssaiot-sc-led",
		.of_compatible = "solidrun,solidsense-aiot-system-controller-led",
	}, {
		.name = "ssaiot-sc-ir",
		.of_compatible = "solidrun,solidsense-aiot-system-controller-ir",
	}, {
		.name = "ssaiot-sc-acc",
		.of_compatible = "solidrun,solidsense-aiot-system-controller-accelerometer",
	}, {
		.name = "ssaiot-sc-gnss",
		.of_compatible = "solidrun,solidsense-aiot-system-controller-gnss",
	}, {
		.name = "ssaiot-sc-charger",
		.of_compatible = "solidrun,solidsense-aiot-system-controller-charger",
	}, {
		.name = "ssaiot-sc-rtc",
		.of_compatible = "solidrun,solidsense-aiot-system-controller-rtc",
	},
};

/**
 * ssaiot_sc_mfd_probe() - Register the sub-devices
 * @dev: System controller device
 *
 * Requires ssaiot_sc_irq_probe() to have run: a sub-device resolves its
 * interrupts as it probes, and finds nothing until the domain is registered.
 */
int ssaiot_sc_mfd_probe(struct device *dev)
{
	int ret;

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, ssaiot_sc_cells,
				   ARRAY_SIZE(ssaiot_sc_cells), NULL, 0, NULL);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add MFD child devices.\n");

	return 0;
}
