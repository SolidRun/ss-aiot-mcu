// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/reboot.h>
#include <linux/version.h>

#include "ssaiot_sc.h"

/* controller cuts power after 1 second */
#define SSAIOT_SC_POWEROFF_DELAY_MS	1000

static void ssaiot_sc_destroy_wq(void *data)
{
	destroy_workqueue(data);
}

static int ssaiot_sc_power_off(struct sys_off_data *data)
{
	struct ssaiot_sc_priv *priv = data->cb_data;
	u8 status;
	int ret;

	ret = ssaiot_sc_xfer(priv, SSAIOT_SC_CMD_SENSOR_OFF,
			     SSAIOT_SC_SENSOR_SOM, NULL, 0, NULL, 0, &status);
	if (ret) {
		dev_emerg(priv->dev, "Failed to request power-off: %d\n", ret);
		return NOTIFY_DONE;
	}

	if (status != SSAIOT_SC_STATUS_OK) {
		/* error status means controller encountered internal error or bug */
		dev_emerg(priv->dev, "Controller refused power-off, status 0x%02x.\n", status);
		return NOTIFY_DONE;
	}

	/* wait with margin for power-off */
	mdelay(2 * SSAIOT_SC_POWEROFF_DELAY_MS);

	dev_emerg(priv->dev, "Controller accepted power-off request, but power is still on.\n");

	return NOTIFY_DONE;
}

static int ssaiot_sc_probe(struct i2c_client *client)
{
	struct ssaiot_sc_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);
	priv->dev = &client->dev;

	priv->irq = fwnode_irq_get(dev_fwnode(priv->dev), 0);
	if (priv->irq < 0)
		return dev_err_probe(priv->dev, priv->irq, "Failed to get irq.\n");

	/*
	 * Freezable so that deferred work stops across suspend. Registered
	 * before the sub-devices, so that devm tears it down only once they are
	 * gone and can no longer queue onto it.
	 */
	priv->wq = alloc_ordered_workqueue("%s", WQ_FREEZABLE,
					   dev_name(priv->dev));
	if (!priv->wq)
		return dev_err_probe(priv->dev, -ENOMEM, "Failed to allocate workqueue.\n");

	ret = devm_add_action_or_reset(priv->dev, ssaiot_sc_destroy_wq,
				       priv->wq);
	if (ret)
		return ret;

	ret = ssaiot_sc_irq_probe(priv->dev);
	if (ret)
		return ret;

	ret = ssaiot_sc_mfd_probe(priv->dev);
	if (ret)
		return ret;

	/* register power-off handler at high priority, to trigger before psci (normal priority) */
	if (of_device_is_system_power_controller(dev_of_node(priv->dev))) {
		ret = devm_register_sys_off_handler(priv->dev, SYS_OFF_MODE_POWER_OFF,
						    SYS_OFF_PRIO_HIGH, ssaiot_sc_power_off, priv);
		if (ret)
			return dev_err_probe(priv->dev, ret,
					     "Failed to register power-off handler.\n");
	}

	dev_info(priv->dev, "SolidSense AIOT System Controller probed.\n");

	return 0;
}

static const struct of_device_id ssaiot_sc_of_match[] = {
	{ .compatible = "solidrun,solidsense-aiot-system-controller" },
	{ },
};
MODULE_DEVICE_TABLE(of, ssaiot_sc_of_match);

static struct i2c_driver ssaiot_sc_driver = {
	.driver = {
		.name = "solidsense-aiot-system-controller",
		.of_match_table = ssaiot_sc_of_match,
	},
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 3, 0)
#pragma error version not supported
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)) && (LINUX_VERSION_CODE < KERNEL_VERSION(6, 5, 0))
	.probe_new = ssaiot_sc_probe,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 5, 0)
	.probe = ssaiot_sc_probe,
#endif
};
module_i2c_driver(ssaiot_sc_driver);

MODULE_AUTHOR("Josua Mayer");
MODULE_DESCRIPTION("SolidRun SolidSense AIOT Board System Controller Driver");
MODULE_LICENSE("GPL v2");
