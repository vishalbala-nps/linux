// SPDX-License-Identifier: GPL-2.0-only
/*
 * Siliconmitus SM5714 fuel gauge driver
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/power_supply.h>

#define SM5714_FG_REG_DEVICE_ID		0x00
#define SM5714_FG_REG_SRAM_RADDR	0x8c
#define SM5714_FG_REG_SRAM_RDATA	0x8d

#define SM5714_FG_SRAM_SOC		0x00
#define SM5714_FG_SRAM_VBAT		0x03
#define SM5714_FG_SRAM_CURRENT		0x05
#define SM5714_FG_SRAM_TEMPERATURE	0x07

struct sm5714_fg {
	struct i2c_client *client;
	struct power_supply *psy;
};

static int sm5714_fg_read_sram(struct sm5714_fg *fg, u8 addr)
{
	int ret;

	ret = i2c_smbus_write_word_data(fg->client, SM5714_FG_REG_SRAM_RADDR, addr);
	if (ret < 0)
		return ret;

	return i2c_smbus_read_word_data(fg->client, SM5714_FG_REG_SRAM_RDATA);
}

static int sm5714_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5714_fg *fg = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC);
		if (ret < 0)
			return ret;
		
		val->intval = min((ret + 0x80) >> 8, 100);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_VBAT);
		if (ret < 0)
			return ret;
		if (ret & 0x8000)
			val->intval = 2700 - (((ret & 0x7fff) * 10) / 109);
		else
			val->intval = ((ret * 10) / 109) + 2700;
		val->intval *= 1000;	
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property sm5714_fg_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static const struct power_supply_desc sm5714_fg_desc = {
	.name		= "sm5714-fuelgauge",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= sm5714_fg_props,
	.num_properties	= ARRAY_SIZE(sm5714_fg_props),
	.get_property	= sm5714_fg_get_property,
};

static int sm5714_fg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = {};
	struct sm5714_fg *fg;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->client = client;
	i2c_set_clientdata(client, fg);
    
	ret = i2c_smbus_read_word_data(client, SM5714_FG_REG_DEVICE_ID);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to read device id\n");

	dev_info(dev, "SM5714 fuel gauge, device id 0x%04x\n", ret);

	cfg.drv_data = fg;
	cfg.fwnode = dev_fwnode(dev);

	fg->psy = devm_power_supply_register(dev, &sm5714_fg_desc, &cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy),
				     "failed to register power supply\n");

	return 0;
}

static const struct of_device_id sm5714_fg_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_fg_of_match);

static struct i2c_driver sm5714_fg_driver = {
	.driver = {
		.name = "sm5714-fuelgauge",
		.of_match_table = sm5714_fg_of_match,
	},
	.probe = sm5714_fg_probe,
};
module_i2c_driver(sm5714_fg_driver);

MODULE_DESCRIPTION("Siliconmitus SM5714 fuel gauge driver");
MODULE_LICENSE("GPL");