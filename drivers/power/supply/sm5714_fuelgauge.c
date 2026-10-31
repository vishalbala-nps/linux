// SPDX-License-Identifier: GPL-2.0-only
/*
 * Siliconmitus SM5714 fuel gauge driver
 */

#include <linux/i2c.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#define SM5714_FG_REG_DEVICE_ID		0x00
#define SM5714_FG_REG_CTRL		0x01
#define SM5714_FG_REG_INTFG		0x02	/* interrupt flags */
#define SM5714_FG_REG_STATUS		0x03
#define SM5714_FG_REG_INTFG_MASK	0x04
#define SM5714_FG_REG_SYSTEM_STATUS	0x10
#define SM5714_FG_REG_TABLE_UNLOCK	0x13
#define SM5714_FG_REG_AUX_CTRL1		0x20
#define SM5714_FG_REG_AUX_CTRL2		0x21
#define SM5714_FG_REG_AUX_STAT		0x22
#define SM5714_FG_REG_SRAM_PROT		0x8b
#define SM5714_FG_REG_SRAM_RADDR	0x8c
#define SM5714_FG_REG_SRAM_RDATA	0x8d
#define SM5714_FG_REG_SRAM_WADDR	0x8e
#define SM5714_FG_REG_SRAM_WDATA	0x8f

#define SM5714_FG_SRAM_SOC		0x00
#define SM5714_FG_SRAM_OCV		0x01
#define SM5714_FG_SRAM_VBAT		0x03
#define SM5714_FG_SRAM_CURRENT		0x05
#define SM5714_FG_SRAM_TEMPERATURE	0x07
#define SM5714_FG_SRAM_VBAT_AVG		0x08
#define SM5714_FG_SRAM_CURRENT_AVG	0x09
/* Battery health estimate, 2048 = 100 % (as decoded by the vendor driver) */
#define SM5714_FG_SRAM_AGING_RATE	0x46

#define SM5714_FG_SOC_FULL		(100 << 8)	/* SOC is 8.8 fixed point */
#define SM5714_FG_SOH_FULL		2048

struct sm5714_fg {
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply_battery_info *bi;	/* NULL without monitored-battery */
	struct mutex sram_lock;	/* SRAM window is a two-step access */
};

/* Returns the 16-bit SRAM word, or a negative error */
static int sm5714_fg_read_sram(struct sm5714_fg *fg, u8 addr)
{
	unsigned int val;
	int ret;

	mutex_lock(&fg->sram_lock);
	ret = regmap_write(fg->regmap, SM5714_FG_REG_SRAM_RADDR, addr);
	if (!ret)
		ret = regmap_read(fg->regmap, SM5714_FG_REG_SRAM_RDATA, &val);
	mutex_unlock(&fg->sram_lock);

	return ret ? ret : val;
}

/* Raw SOC, 8.8 fixed point percent, capped at 100.00 % */
static int sm5714_fg_get_soc_raw(struct sm5714_fg *fg)
{
	int ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC);

	return ret < 0 ? ret : min(ret, SM5714_FG_SOC_FULL);
}

/* Raw health estimate, 2048 = 100 %, capped at 100 % */
static int sm5714_fg_get_soh_raw(struct sm5714_fg *fg)
{
	int ret = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_AGING_RATE);

	return ret < 0 ? ret : min(ret, SM5714_FG_SOH_FULL);
}

/* Present full capacity: design capacity scaled by health */
static int sm5714_fg_get_charge_full(struct sm5714_fg *fg, int *uah)
{
	int soh;

	if (!fg->bi || fg->bi->charge_full_design_uah <= 0)
		return -ENODATA;

	soh = sm5714_fg_get_soh_raw(fg);
	if (soh < 0)
		return soh;

	*uah = mult_frac(fg->bi->charge_full_design_uah, soh, SM5714_FG_SOH_FULL);
	return 0;
}

static int sm5714_fg_conv_voltage(int raw)
{
	if (raw & 0x8000)
		return 2700 - (((raw & 0x7fff) * 10) / 109);

	return ((raw * 10) / 109) + 2700;
}

static int sm5714_fg_conv_current(struct sm5714_fg *fg, int raw)
{
	unsigned int aux;
	int curr;

	curr = ((raw & 0x7fff) * 1000) / 2044;
	if (raw & 0x8000)
		curr = -curr;

	/* AUX_STAT bit 10 selects the low-gain range: scale by 2.5 */
	if (!regmap_read(fg->regmap, SM5714_FG_REG_AUX_STAT, &aux) &&
	    (aux & BIT(10)))
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
	int ret, curr, full;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = sm5714_fg_get_soc_raw(fg);
		if (ret < 0)
			return ret;
		/* chip reports 8.8 fixed point; round to whole percent */
		val->intval = min((ret + 0x80) >> 8, 100);
		break;

	case POWER_SUPPLY_PROP_STATE_OF_HEALTH:
		ret = sm5714_fg_get_soh_raw(fg);
		if (ret < 0)
			return ret;
		val->intval = ret * 100 / SM5714_FG_SOH_FULL;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (!fg->bi || fg->bi->charge_full_design_uah <= 0)
			return -ENODATA;
		val->intval = fg->bi->charge_full_design_uah;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = sm5714_fg_get_charge_full(fg, &full);
		if (ret < 0)
			return ret;
		val->intval = full;
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = sm5714_fg_get_charge_full(fg, &full);
		if (ret < 0)
			return ret;
		ret = sm5714_fg_get_soc_raw(fg);
		if (ret < 0)
			return ret;
		val->intval = mult_frac(full, ret, SM5714_FG_SOC_FULL);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (!fg->bi || fg->bi->voltage_min_design_uv <= 0)
			return -ENODATA;
		val->intval = fg->bi->voltage_min_design_uv;
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

static const enum power_supply_property sm5714_fg_props[] = {
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
	POWER_SUPPLY_PROP_STATE_OF_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static const struct power_supply_desc sm5714_fg_desc = {
	.name		= "sm5714-fuelgauge",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= sm5714_fg_props,
	.num_properties	= ARRAY_SIZE(sm5714_fg_props),
	.get_property	= sm5714_fg_get_property,
	.external_power_changed = power_supply_changed,
};

static bool sm5714_fg_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SM5714_FG_REG_DEVICE_ID ... SM5714_FG_REG_INTFG_MASK:
	case SM5714_FG_REG_SYSTEM_STATUS:
	case SM5714_FG_REG_TABLE_UNLOCK:
	case SM5714_FG_REG_AUX_CTRL1 ... SM5714_FG_REG_AUX_STAT:
	case SM5714_FG_REG_SRAM_PROT ... SM5714_FG_REG_SRAM_WDATA:
		return true;
	default:
		return false;
	}
}

/* Only the SRAM read address is written by this driver */
static bool sm5714_fg_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg == SM5714_FG_REG_SRAM_RADDR;
}

/*
 * Keep debugfs dumps away from the interrupt flags (likely clear-on-read)
 * and from the SRAM data window, whose meaning depends on the address
 * last written.
 */
static bool sm5714_fg_precious_reg(struct device *dev, unsigned int reg)
{
	return reg == SM5714_FG_REG_INTFG || reg == SM5714_FG_REG_SRAM_RDATA;
}

/* 16-bit registers, low byte first on the wire (as SMBus word access) */
static const struct regmap_config sm5714_fg_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.max_register = SM5714_FG_REG_SRAM_WDATA,
	.readable_reg = sm5714_fg_readable_reg,
	.writeable_reg = sm5714_fg_writeable_reg,
	.precious_reg = sm5714_fg_precious_reg,
};

static int sm5714_fg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = {};
	struct sm5714_fg *fg;
	unsigned int id;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &fg->sram_lock);
	if (ret)
		return ret;

	fg->regmap = devm_regmap_init_i2c(client, &sm5714_fg_regmap_config);
	if (IS_ERR(fg->regmap))
		return dev_err_probe(dev, PTR_ERR(fg->regmap), "regmap init failed\n");

	ret = regmap_read(fg->regmap, SM5714_FG_REG_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device id\n");

	dev_info(dev, "SM5714 fuel gauge, device id 0x%04x\n", id);

	cfg.drv_data = fg;
	cfg.fwnode = dev_fwnode(dev);

	fg->psy = devm_power_supply_register(dev, &sm5714_fg_desc, &cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy),
				     "failed to register power supply\n");

	/* Optional: design capacity etc. from a simple-battery node */
	if (power_supply_get_battery_info(fg->psy, &fg->bi)) {
		dev_info(dev, "no monitored-battery, capacity properties unavailable\n");
		fg->bi = NULL;
	}

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