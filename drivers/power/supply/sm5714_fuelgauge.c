// SPDX-License-Identifier: GPL-2.0-only
/*
 * Siliconmitus SM5714 fuel gauge driver
 */

#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/power_supply.h>

#define SM5714_FG_REG_DEVICE_ID		0x00
#define SM5714_FG_REG_AUX_STAT		0x22
#define SM5714_FG_REG_SRAM_RADDR	0x8c
#define SM5714_FG_REG_SRAM_RDATA	0x8d

#define SM5714_FG_SRAM_SOC		0x00
#define SM5714_FG_SRAM_OCV		0x01
#define SM5714_FG_SRAM_VBAT		0x03
#define SM5714_FG_SRAM_CURRENT		0x05
#define SM5714_FG_SRAM_TEMPERATURE	0x07
#define SM5714_FG_SRAM_VBAT_AVG		0x08
#define SM5714_FG_SRAM_CURRENT_AVG	0x09

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

static int sm5714_fg_conv_voltage(int raw)
{
	if (raw & 0x8000)
		return 2700 - (((raw & 0x7fff) * 10) / 109);

	return ((raw * 10) / 109) + 2700;
}

static int sm5714_fg_conv_current(struct sm5714_fg *fg, int raw)
{
	int curr, aux;

	curr = ((raw & 0x7fff) * 1000) / 2044;
	if (raw & 0x8000)
		curr = -curr;

	/* AUX_STAT bit 10 selects the low-gain range: scale by 2.5 */
	aux = i2c_smbus_read_word_data(fg->client, SM5714_FG_REG_AUX_STAT);
	if (aux >= 0 && (aux & BIT(10)))
		curr = curr * 25 / 10;

	return curr;
}

static int sm5714_fg_get_current(struct sm5714_fg *fg, u8 sram_addr, int *out)
{
	int ret;

	ret = sm5714_fg_read_sram(fg, sram_addr);
	if (ret < 0)
		return ret;

	*out = sm5714_fg_conv_current(fg, ret);

	return 0;
}

static int sm5714_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5714_fg *fg = power_supply_get_drvdata(psy);
	int ret, curr;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC);
		if (ret < 0)
			return ret;
		/* chip reports 8.8 fixed point; round to whole percent */
		val->intval = min((ret + 0x80) >> 8, 100);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_VBAT);
		if (ret < 0)
			return ret;
		val->intval = sm5714_fg_conv_voltage(ret) * 1000;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_VBAT_AVG);
		if (ret < 0)
			return ret;
		val->intval = sm5714_fg_conv_voltage(ret) * 1000;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_OCV);
		if (ret < 0)
			return ret;
		val->intval = ((ret * 1000) >> 11) * 1000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sm5714_fg_get_current(fg, SM5714_FG_SRAM_CURRENT, &curr);
		if (ret < 0)
			return ret;
		val->intval = curr * 1000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = sm5714_fg_get_current(fg, SM5714_FG_SRAM_CURRENT_AVG, &curr);
		if (ret < 0)
			return ret;
		val->intval = curr * 1000;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_TEMPERATURE);
		if (ret < 0)
			return ret;
		val->intval = (((ret & 0x7fff) * 10) * 2989) >> 11 >> 8;
		if (ret & 0x8000)
			val->intval = -val->intval;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		if (power_supply_get_property_from_supplier(psy,
				POWER_SUPPLY_PROP_STATUS, val))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property sm5714_fg_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_STATUS,
};

static const struct power_supply_desc sm5714_fg_desc = {
	.name		= "sm5714-fuelgauge",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= sm5714_fg_props,
	.num_properties	= ARRAY_SIZE(sm5714_fg_props),
	.get_property	= sm5714_fg_get_property,
	.external_power_changed = power_supply_changed,
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