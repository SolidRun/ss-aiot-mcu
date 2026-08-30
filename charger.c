// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SolidRun SolidSense AIOT Board System Controller Charger Driver
 *
 * Copyright (C) 2026 Josua Mayer <josua@solid-run.com>
 */

#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#include "ssaiot_sc.h"

/*
 * Response payload of CMD_SENSOR_READ / SENSOR_CHARGER. The controller
 * serialises each measurement little endian, low byte first, and ibat is two's
 * complement as the charger reports it.
 *
 * The order here is the order on the wire, which is not the order the firmware
 * declares its own status structure in - it writes vbat before vbus.
 */
struct ssaiot_sc_charger_state {
	u8 flags;
	__le16 ibat;	/* battery current, mA, -10000 to +5025 */
	__le16 vbat;	/* battery voltage, mV, 0 to 5000 */
	__le16 vbus;	/* supply voltage, mV, 0 to 20000 */
} __packed;

#define SSAIOT_SC_CHARGER_F_SUPPLY	BIT(0)	/* J1 DC jack or J3 USB */
#define SSAIOT_SC_CHARGER_F_CHARGING	BIT(1)
#define SSAIOT_SC_CHARGER_F_VBUS_FAULT	BIT(2)
#define SSAIOT_SC_CHARGER_F_BAT_FAULT	BIT(3)

/* there is no temperature sensor, so an OCV table is read at its room entry */
#define SSAIOT_SC_CHARGER_OCV_TEMP_C	20

struct ssaiot_sc_charger {
	struct ssaiot_sc_priv *sc;
	struct power_supply *battery;
	struct power_supply_battery_info *info;
};

static int ssaiot_sc_charger_read(struct ssaiot_sc_priv *sc,
				  struct ssaiot_sc_charger_state *state)
{
	u8 status;
	int ret;

	ret = ssaiot_sc_xfer(sc, SSAIOT_SC_CMD_SENSOR_READ,
			     SSAIOT_SC_SENSOR_CHARGER, NULL, 0,
			     (u8 *)state, sizeof(*state), &status);
	if (ret)
		return ret;

	/* check status, data is invalid on error */
	if (status != SSAIOT_SC_STATUS_OK)
		return -ENODATA;

	return 0;
}

/* Translate charger flags to power-supply status */
static int ssaiot_sc_charger_flags_to_psp_status(u8 charger_flags)
{
	if (charger_flags & SSAIOT_SC_CHARGER_F_CHARGING)
		return POWER_SUPPLY_STATUS_CHARGING;
	else if (charger_flags & SSAIOT_SC_CHARGER_F_SUPPLY)
		/* mains online but not charging*/
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		/* mains offline, discharging */
		return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int ssaiot_sc_charger_battery_get_property(struct power_supply *psy,
						  enum power_supply_property psp,
						  union power_supply_propval *val)
{
	struct ssaiot_sc_charger *chg = power_supply_get_drvdata(psy);
	struct ssaiot_sc_priv *sc = chg->sc;
	struct ssaiot_sc_charger_state state;
	int vbat_oc;
	int ret;

	/* handle constant properties first */
	switch (psp) {
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		/* always report present, no runtime detection */
		val->intval = 1;
		return 0;
	default:
		break;
	}

	/* read charger status */
	ret = ssaiot_sc_charger_read(sc, &state);
	if (ret)
		return ret;

	/* handle dynamic properties */
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = ssaiot_sc_charger_flags_to_psp_status(state.flags);
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (state.flags & SSAIOT_SC_CHARGER_F_BAT_FAULT)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* charger value in mV, property value in uV */
		val->intval = le16_to_cpu(state.vbat) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* charger value in mA, property value in uA, signed */
		val->intval = (s16)le16_to_cpu(state.ibat) * 1000;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* no battery info, can't calculate capacity */
		if (!chg->info)
			return -ENODATA;

		/* ocv table describes open-circuit voltage, but controller
		 * reports terminal voltage. Apply IR correction using ibat
		 * if available.
		 */
		vbat_oc = le16_to_cpu(state.vbat) * 1000;
		if (chg->info->factory_internal_resistance_uohm > 0)
			vbat_oc -= (s16)le16_to_cpu(state.ibat) *
				   (chg->info->factory_internal_resistance_uohm / 1000);

		/* derive capacity from ocv table */
		ret = power_supply_batinfo_ocv2cap(chg->info, vbat_oc, SSAIOT_SC_CHARGER_OCV_TEMP_C);
		if (ret < 0)
			return ret;

		val->intval = ret;
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property ssaiot_sc_charger_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_CAPACITY,
};

/* battery component of charger */
static const struct power_supply_desc ssaiot_sc_charger_battery_desc = {
	.name = "ssaiot-sc-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = ssaiot_sc_charger_battery_props,
	.num_properties = ARRAY_SIZE(ssaiot_sc_charger_battery_props),
	.get_property = ssaiot_sc_charger_battery_get_property,
};

static int ssaiot_sc_charger_ac_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct ssaiot_sc_charger *chg = power_supply_get_drvdata(psy);
	struct ssaiot_sc_charger_state state;
	int ret;

	/* handle constant properties first */
	switch (psp) {
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		return 0;
	default:
		break;
	}

	/* read charger status */
	ret = ssaiot_sc_charger_read(chg->sc, &state);
	if (ret)
		return ret;

	/* handle dynamic properties */
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(state.flags & SSAIOT_SC_CHARGER_F_SUPPLY);
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (state.flags & SSAIOT_SC_CHARGER_F_VBUS_FAULT)
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* charger value in mV, property value in uV */
		val->intval = le16_to_cpu(state.vbus) * 1000;
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property ssaiot_sc_charger_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_SCOPE,
};

/* mains component of charger advertised as generic "ac" */
static const struct power_supply_desc ssaiot_sc_charger_ac_desc = {
	.name = "ssaiot-sc-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = ssaiot_sc_charger_ac_props,
	.num_properties = ARRAY_SIZE(ssaiot_sc_charger_ac_props),
	.get_property = ssaiot_sc_charger_ac_get_property,
};

static void ssaiot_sc_charger_put_info(void *data)
{
	struct ssaiot_sc_charger *chg = data;

	power_supply_put_battery_info(chg->battery, chg->info);
}

static int ssaiot_sc_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = { };
	struct ssaiot_sc_charger *chg;
	struct power_supply *psy;
	int ret;

	/* the mfd cell has no dedicated dt node, reuse parent */
	dev->of_node = dev->parent->of_node;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->sc = dev_get_drvdata(dev->parent);
	cfg.drv_data = chg;
	cfg.of_node = dev->of_node;

	chg->battery = devm_power_supply_register(dev,
						  &ssaiot_sc_charger_battery_desc,
						  &cfg);
	if (IS_ERR(chg->battery))
		return dev_err_probe(dev, PTR_ERR(chg->battery),
				     "Failed to register battery supply.\n");

	/* get optional battery-info for ocv table based capacity calculation */
	ret = power_supply_get_battery_info(chg->battery, &chg->info);
	if (ret == -ENODEV) {
		dev_info(dev, "monitored-battery missing, can't report capacity\n");
	} else if (ret) {
		return dev_err_probe(dev, ret, "Failed to read battery info.\n");
	} else {
		ret = devm_add_action_or_reset(dev, ssaiot_sc_charger_put_info, chg);
		if (ret) {
			power_supply_put_battery_info(chg->battery, chg->info);
			return ret;
		}
	}

	psy = devm_power_supply_register(dev, &ssaiot_sc_charger_ac_desc, &cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy),
				     "Failed to register mains supply.\n");

	return 0;
}

static const struct platform_device_id ssaiot_sc_charger_id_table[] = {
	{ "ssaiot-sc-charger", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, ssaiot_sc_charger_id_table);

static struct platform_driver ssaiot_sc_charger_driver = {
	.driver = {
		.name = "solidsense-aiot-system-controller-charger",
	},
	.probe = ssaiot_sc_charger_probe,
	.id_table = ssaiot_sc_charger_id_table,
};
module_platform_driver(ssaiot_sc_charger_driver);

MODULE_AUTHOR("Josua Mayer");
MODULE_DESCRIPTION("SolidRun SolidSense AIOT Board System Controller Charger Driver");
MODULE_LICENSE("GPL v2");
