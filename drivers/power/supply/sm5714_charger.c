// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 switching charger
 *
 * Register layout and field encodings come from Samsung's downstream
 * SM5714 charger driver and MFD headers.
 *
 * Deliberately small first version:
 *   - turns off the charger watchdog (nothing kicks it yet),
 *   - enables hardware AICL at 4.5 V,
 *   - programs charge current, float voltage and top-off current from the
 *     monitored-battery node, plus the input current limit,
 *   - enables charging, and re-applies all of it on cable insertion,
 *   - polls status, because interrupts go through the shared SM5714
 *     IRQ line which is not wired up yet.
 * No charger-type detection, no fast charging, no OTG, no battery
 * temperature gating.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define SM5714_CHG_STATUS1		0x0d
#define  SM5714_STATUS1_VBUSPOK		BIT(0)
#define  SM5714_STATUS1_VBUSUVLO	BIT(1)
#define  SM5714_STATUS1_VBUSOVP		BIT(2)
#define SM5714_CHG_STATUS2		0x0e
#define  SM5714_STATUS2_AICL		BIT(0)
#define  SM5714_STATUS2_NOBAT		BIT(2)
#define  SM5714_STATUS2_CHGON		BIT(3)
#define  SM5714_STATUS2_TOPOFF		BIT(5)
#define  SM5714_STATUS2_WDTOFF		BIT(7)
#define SM5714_CHG_CNTL1		0x13
#define  SM5714_CNTL1_ENQ4FET		BIT(3)	/* charge enable */
#define  SM5714_CNTL1_AICLEN		BIT(6)
#define SM5714_CHG_CNTL2		0x14
#define  SM5714_CNTL2_OPMODE		GENMASK(3, 0)
#define  SM5714_OPMODE_SUSPEND		0x0
#define  SM5714_OPMODE_CHG_ON		0x5
#define SM5714_CHG_VBUSCNTL		0x15
#define  SM5714_VBUSCNTL_LIMIT		GENMASK(6, 0)
#define SM5714_CHG_CHGCNTL2		0x18	/* fast-charge current, 8 bits */
#define SM5714_CHG_CHGCNTL4		0x1a
#define  SM5714_CHGCNTL4_BATREG		GENMASK(5, 0)
#define  SM5714_CHGCNTL4_AUTOSTOP	BIT(6)
#define SM5714_CHG_CHGCNTL5		0x1b
#define  SM5714_CHGCNTL5_TOPOFF		GENMASK(4, 0)
#define SM5714_CHG_WDTCNTL		0x22
#define  SM5714_WDTCNTL_EN		BIT(0)
#define  SM5714_WDTCNTL_CLR_EXPIRED	BIT(6)
#define SM5714_CHG_FACTORY1		0x25
#define  SM5714_FACTORY1_FACMODE	BIT(0)
#define SM5714_CHG_CHGCNTL11		0x46
#define  SM5714_CHGCNTL11_AICL_TH	GENMASK(7, 6)
#define  SM5714_AICL_TH_4500MV		0x2
#define SM5714_CHG_DEVICEID		0x50

/* Input current limit: 100 mA + 25 mA * n, n = 0..127 */
#define SM5714_ILIM_MIN_UA		100000
#define SM5714_ILIM_STEP_UA		25000
#define SM5714_ILIM_MAX_UA		(SM5714_ILIM_MIN_UA + 127 * SM5714_ILIM_STEP_UA)

/* Fast-charge current: 109.375 mA + 15.625 mA * (n - 7), n = 7..0xe0 */
#define SM5714_ICHG_MIN_UA		109375
#define SM5714_ICHG_STEP_UA		15625
#define SM5714_ICHG_MAX_UA		3500000

/* Top-off current: 100 mA + 25 mA * n, saturates at 800 mA (n >= 0x1c) */
#define SM5714_TOPOFF_MAX_UA		800000

#define SM5714_DEFAULT_ILIM_UA		1500000
/* The bootloader leaves the charge current at its 109 mA minimum */
#define SM5714_DEFAULT_ICHG_UA		1000000
#define SM5714_POLL_MS			2000

struct sm5714_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work poll_work;
	struct mutex lock;		/* serialises limit changes and enable ramp */
	u32 ilim_ua;
	u32 ichg_ua;
	u32 ichg_max_ua;		/* from DT; sysfs writes are capped here */
	u32 float_uv;			/* 0: keep bootloader value */
	unsigned int float_reg;
	u32 topoff_ua;			/* 0: keep bootloader value */
	unsigned int last_st1;
	unsigned int last_st2;
};

/* ---- field conversions -------------------------------------------------- */

static unsigned int sm5714_ilim_to_reg(u32 ua)
{
	ua = clamp_t(u32, ua, SM5714_ILIM_MIN_UA, SM5714_ILIM_MAX_UA);
	return (ua - SM5714_ILIM_MIN_UA) / SM5714_ILIM_STEP_UA;
}

static int sm5714_reg_to_ilim(unsigned int reg)
{
	return SM5714_ILIM_MIN_UA + (reg & SM5714_VBUSCNTL_LIMIT) * SM5714_ILIM_STEP_UA;
}

static unsigned int sm5714_ichg_to_reg(u32 ua)
{
	ua = clamp_t(u32, ua, SM5714_ICHG_MIN_UA, SM5714_ICHG_MAX_UA);
	return 7 + (ua - SM5714_ICHG_MIN_UA) / SM5714_ICHG_STEP_UA;
}

static int sm5714_reg_to_ichg(unsigned int reg)
{
	if (reg <= 7)
		return SM5714_ICHG_MIN_UA;
	if (reg >= 0xe0)
		return SM5714_ICHG_MAX_UA;
	return SM5714_ICHG_MIN_UA + (reg - 7) * SM5714_ICHG_STEP_UA;
}

/*
 * BATREG is piecewise:
 *   n = 0..3   3.70 .. 3.85 V in 50 mV steps
 *   n = 4..5   3.90 .. 4.00 V in 100 mV steps
 *   n = 6..63  4.05 .. 4.62 V in 10 mV steps
 * Rounds down, so a requested value is never exceeded.
 */
static int sm5714_batreg_to_reg(u32 uv, unsigned int *reg)
{
	if (uv < 3700000 || uv > 4620000)
		return -EINVAL;

	if (uv < 3900000)
		*reg = (uv - 3700000) / 50000;
	else if (uv < 4050000)
		*reg = 4 + (uv - 3900000) / 100000;
	else
		*reg = 6 + (uv - 4050000) / 10000;

	return 0;
}

static int sm5714_reg_to_batreg(unsigned int reg)
{
	reg &= SM5714_CHGCNTL4_BATREG;
	if (reg <= 3)
		return 3700000 + reg * 50000;
	if (reg <= 5)
		return 3900000 + (reg - 4) * 100000;
	return 4050000 + (reg - 6) * 10000;
}

static unsigned int sm5714_topoff_to_reg(u32 ua)
{
	ua = clamp_t(u32, ua, 100000, SM5714_TOPOFF_MAX_UA);
	return (ua - 100000) / 25000;
}

static int sm5714_reg_to_topoff(unsigned int reg)
{
	reg &= SM5714_CHGCNTL5_TOPOFF;
	if (reg >= 0x1c)
		return SM5714_TOPOFF_MAX_UA;
	return 100000 + reg * 25000;
}

/* ---- hardware operations ------------------------------------------------ */

static int sm5714_set_ilim(struct sm5714_charger *chg, u32 ua)
{
	return regmap_update_bits(chg->regmap, SM5714_CHG_VBUSCNTL,
				  SM5714_VBUSCNTL_LIMIT,
				  FIELD_PREP(SM5714_VBUSCNTL_LIMIT,
					     sm5714_ilim_to_reg(ua)));
}

/*
 * Downstream never flips ENQ4FET with a high input limit: it drops the
 * limit to 500 mA, waits 1 ms per 250 mA of difference, enables, then
 * restores the limit. Keep that to avoid an input current step.
 * Caller holds chg->lock.
 */
static int sm5714_enable_charging(struct sm5714_charger *chg)
{
	int ret;

	ret = regmap_update_bits(chg->regmap, SM5714_CHG_CNTL2,
				 SM5714_CNTL2_OPMODE,
				 FIELD_PREP(SM5714_CNTL2_OPMODE, SM5714_OPMODE_CHG_ON));
	if (ret)
		return ret;

	if (chg->ilim_ua > 500000) {
		ret = sm5714_set_ilim(chg, 500000);
		if (ret)
			return ret;
		msleep((chg->ilim_ua - 500000) / 250000);
	}

	ret = regmap_set_bits(chg->regmap, SM5714_CHG_CNTL1, SM5714_CNTL1_ENQ4FET);
	if (ret)
		return ret;

	return sm5714_set_ilim(chg, chg->ilim_ua);
}

/*
 * Program the battery-side limits. Called before every charge enable,
 * at probe and on cable insertion. Caller holds chg->lock.
 */
static int sm5714_apply_limits(struct sm5714_charger *chg)
{
	struct regmap *map = chg->regmap;
	int ret;

	if (chg->float_uv) {
		ret = regmap_update_bits(map, SM5714_CHG_CHGCNTL4,
					 SM5714_CHGCNTL4_BATREG,
					 FIELD_PREP(SM5714_CHGCNTL4_BATREG,
						    chg->float_reg));
		if (ret)
			return ret;
	}

	ret = regmap_write(map, SM5714_CHG_CHGCNTL2,
			   sm5714_ichg_to_reg(chg->ichg_ua));
	if (ret)
		return ret;

	if (chg->topoff_ua) {
		ret = regmap_update_bits(map, SM5714_CHG_CHGCNTL5,
					 SM5714_CHGCNTL5_TOPOFF,
					 FIELD_PREP(SM5714_CHGCNTL5_TOPOFF,
						    sm5714_topoff_to_reg(chg->topoff_ua)));
		if (ret)
			return ret;
	}

	/* Stop charging at top-off instead of holding the battery at CV */
	return regmap_set_bits(map, SM5714_CHG_CHGCNTL4, SM5714_CHGCNTL4_AUTOSTOP);
}

static void sm5714_log_config(struct sm5714_charger *chg)
{
	unsigned int ilim, ichg, batreg, topoff;

	if (regmap_read(chg->regmap, SM5714_CHG_VBUSCNTL, &ilim) ||
	    regmap_read(chg->regmap, SM5714_CHG_CHGCNTL2, &ichg) ||
	    regmap_read(chg->regmap, SM5714_CHG_CHGCNTL4, &batreg) ||
	    regmap_read(chg->regmap, SM5714_CHG_CHGCNTL5, &topoff))
		return;

	dev_info(chg->dev,
		 "input limit %d mA, charge current %d mA, float %d mV, top-off %d mA\n",
		 sm5714_reg_to_ilim(ilim) / 1000, sm5714_reg_to_ichg(ichg) / 1000,
		 sm5714_reg_to_batreg(batreg) / 1000,
		 sm5714_reg_to_topoff(topoff) / 1000);
}

static int sm5714_hw_init(struct sm5714_charger *chg)
{
	struct regmap *map = chg->regmap;
	unsigned int val;
	int ret;

	ret = regmap_read(map, SM5714_CHG_DEVICEID, &val);
	if (ret)
		return dev_err_probe(chg->dev, ret, "cannot read device id\n");
	dev_info(chg->dev, "SM5714 charger, rev %u\n", val & 0x3);

	/*
	 * Nothing here kicks the charger watchdog yet, so turn it off and
	 * clear a stale expiry left by the bootloader; an expired watchdog
	 * stops charging.
	 */
	ret = regmap_clear_bits(map, SM5714_CHG_WDTCNTL, SM5714_WDTCNTL_EN);
	if (ret)
		return ret;
	ret = regmap_read(map, SM5714_CHG_STATUS2, &val);
	if (ret)
		return ret;
	if (val & SM5714_STATUS2_WDTOFF) {
		dev_info(chg->dev, "clearing expired charger watchdog\n");
		ret = regmap_set_bits(map, SM5714_CHG_WDTCNTL,
				      SM5714_WDTCNTL_CLR_EXPIRED);
		if (ret)
			return ret;
	}

	/* Hardware AICL: back off input current if VBUS sags below 4.5 V */
	ret = regmap_update_bits(map, SM5714_CHG_CHGCNTL11, SM5714_CHGCNTL11_AICL_TH,
				 FIELD_PREP(SM5714_CHGCNTL11_AICL_TH,
					    SM5714_AICL_TH_4500MV));
	if (ret)
		return ret;
	ret = regmap_set_bits(map, SM5714_CHG_CNTL1, SM5714_CNTL1_AICLEN);
	if (ret)
		return ret;

	mutex_lock(&chg->lock);
	ret = sm5714_apply_limits(chg);
	if (!ret)
		ret = sm5714_enable_charging(chg);
	mutex_unlock(&chg->lock);
	if (ret)
		return ret;

	sm5714_log_config(chg);
	return 0;
}

/* ---- power_supply class ------------------------------------------------- */

static int sm5714_get_status(struct sm5714_charger *chg, int *status)
{
	unsigned int st1, st2;
	int ret;

	ret = regmap_read(chg->regmap, SM5714_CHG_STATUS1, &st1);
	if (ret)
		return ret;
	ret = regmap_read(chg->regmap, SM5714_CHG_STATUS2, &st2);
	if (ret)
		return ret;

	if (st2 & SM5714_STATUS2_TOPOFF)
		*status = POWER_SUPPLY_STATUS_FULL;
	else if (st2 & SM5714_STATUS2_CHGON)
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else if (st1 & SM5714_STATUS1_VBUSPOK)
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		*status = POWER_SUPPLY_STATUS_DISCHARGING;

	return 0;
}

static int sm5714_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct sm5714_charger *chg = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chg->regmap, SM5714_CHG_STATUS1, &reg);
		if (ret)
			return ret;
		val->intval = !!(reg & SM5714_STATUS1_VBUSPOK);
		return 0;

	case POWER_SUPPLY_PROP_STATUS:
		return sm5714_get_status(chg, &val->intval);

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = regmap_read(chg->regmap, SM5714_CHG_STATUS2, &reg);
		if (ret)
			return ret;
		val->intval = (reg & SM5714_STATUS2_CHGON) ?
			      POWER_SUPPLY_CHARGE_TYPE_FAST :
			      POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = regmap_read(chg->regmap, SM5714_CHG_VBUSCNTL, &reg);
		if (ret)
			return ret;
		val->intval = sm5714_reg_to_ilim(reg);
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(chg->regmap, SM5714_CHG_CHGCNTL2, &reg);
		if (ret)
			return ret;
		val->intval = sm5714_reg_to_ichg(reg);
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = regmap_read(chg->regmap, SM5714_CHG_CHGCNTL4, &reg);
		if (ret)
			return ret;
		val->intval = sm5714_reg_to_batreg(reg);
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = regmap_read(chg->regmap, SM5714_CHG_CHGCNTL5, &reg);
		if (ret)
			return ret;
		val->intval = sm5714_reg_to_topoff(reg);
		return 0;

	default:
		return -EINVAL;
	}
}

static int sm5714_set_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       const union power_supply_propval *val)
{
	struct sm5714_charger *chg = power_supply_get_drvdata(psy);
	int ret;

	if (val->intval <= 0)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		mutex_lock(&chg->lock);
		chg->ilim_ua = clamp_t(u32, val->intval,
				       SM5714_ILIM_MIN_UA, SM5714_ILIM_MAX_UA);
		ret = sm5714_set_ilim(chg, chg->ilim_ua);
		mutex_unlock(&chg->lock);
		return ret;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		mutex_lock(&chg->lock);
		chg->ichg_ua = min_t(u32, val->intval, chg->ichg_max_ua);
		ret = regmap_write(chg->regmap, SM5714_CHG_CHGCNTL2,
				   sm5714_ichg_to_reg(chg->ichg_ua));
		mutex_unlock(&chg->lock);
		return ret;

	default:
		return -EINVAL;
	}
}

static int sm5714_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT ||
	       psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
}

static const enum power_supply_property sm5714_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static const struct power_supply_desc sm5714_desc = {
	.name = "sm5714-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm5714_props,
	.num_properties = ARRAY_SIZE(sm5714_props),
	.get_property = sm5714_get_property,
	.set_property = sm5714_set_property,
	.property_is_writeable = sm5714_property_is_writeable,
};

/* ---- status polling ----------------------------------------------------- */

#define SM5714_ST1_WATCH	(SM5714_STATUS1_VBUSPOK | SM5714_STATUS1_VBUSOVP)
#define SM5714_ST2_WATCH	(SM5714_STATUS2_CHGON | SM5714_STATUS2_TOPOFF | \
				 SM5714_STATUS2_WDTOFF | SM5714_STATUS2_NOBAT)

static void sm5714_poll_work(struct work_struct *work)
{
	struct sm5714_charger *chg = container_of(to_delayed_work(work),
						  struct sm5714_charger, poll_work);
	unsigned int st1, st2;
	bool changed;

	if (regmap_read(chg->regmap, SM5714_CHG_STATUS1, &st1) ||
	    regmap_read(chg->regmap, SM5714_CHG_STATUS2, &st2))
		goto out;

	/*
	 * On cable insertion re-apply all limits and the charge enable.
	 * Unknown whether VBUS removal resets them; this is cheap insurance.
	 */
	if ((st1 & SM5714_STATUS1_VBUSPOK) &&
	    !(chg->last_st1 & SM5714_STATUS1_VBUSPOK)) {
		mutex_lock(&chg->lock);
		if (sm5714_apply_limits(chg) || sm5714_enable_charging(chg))
			dev_warn(chg->dev, "failed to re-enable charging\n");
		mutex_unlock(&chg->lock);
	}

	changed = ((st1 ^ chg->last_st1) & SM5714_ST1_WATCH) ||
		  ((st2 ^ chg->last_st2) & SM5714_ST2_WATCH);
	chg->last_st1 = st1;
	chg->last_st2 = st2;

	if (changed)
		power_supply_changed(chg->psy);
out:
	schedule_delayed_work(&chg->poll_work, msecs_to_jiffies(SM5714_POLL_MS));
}

/* ---- probe -------------------------------------------------------------- */

static int sm5714_parse_battery(struct sm5714_charger *chg,
				struct power_supply_battery_info *bi)
{
	struct device *dev = chg->dev;
	int ret;

	chg->ichg_ua = SM5714_DEFAULT_ICHG_UA;

	if (!bi) {
		dev_warn(dev, "no monitored-battery: charging at %u mA, float voltage and top-off left as the bootloader set them\n",
			 chg->ichg_ua / 1000);
		chg->ichg_max_ua = chg->ichg_ua;
		return 0;
	}

	if (bi->constant_charge_voltage_max_uv > 0) {
		ret = sm5714_batreg_to_reg(bi->constant_charge_voltage_max_uv,
					   &chg->float_reg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "unsupported float voltage %d uV\n",
					     bi->constant_charge_voltage_max_uv);
		chg->float_uv = bi->constant_charge_voltage_max_uv;
	} else {
		dev_warn(dev, "no float voltage in DT, keeping bootloader value\n");
	}

	if (bi->constant_charge_current_max_ua > 0)
		chg->ichg_ua = bi->constant_charge_current_max_ua;
	else
		dev_warn(dev, "no charge current in DT, using %u mA\n",
			 chg->ichg_ua / 1000);
	chg->ichg_max_ua = chg->ichg_ua;

	if (bi->charge_term_current_ua > 0)
		chg->topoff_ua = bi->charge_term_current_ua;

	return 0;
}

static const struct regmap_config sm5714_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SM5714_CHG_DEVICEID,
};

static int sm5714_charger_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct power_supply_battery_info *bi = NULL;
	struct sm5714_charger *chg;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;

	chg->regmap = devm_regmap_init_i2c(client, &sm5714_regmap_config);
	if (IS_ERR(chg->regmap))
		return dev_err_probe(dev, PTR_ERR(chg->regmap), "regmap init failed\n");

	chg->ilim_ua = SM5714_DEFAULT_ILIM_UA;
	device_property_read_u32(dev, "input-current-limit-microamp", &chg->ilim_ua);
	chg->ilim_ua = clamp_t(u32, chg->ilim_ua, SM5714_ILIM_MIN_UA, SM5714_ILIM_MAX_UA);

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(dev);

	chg->psy = devm_power_supply_register(dev, &sm5714_desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "failed to register power supply\n");

	/* From a simple-battery node via monitored-battery */
	if (power_supply_get_battery_info(chg->psy, &bi))
		bi = NULL;

	ret = sm5714_parse_battery(chg, bi);
	if (ret)
		return ret;

	ret = sm5714_hw_init(chg);
	if (ret)
		return dev_err_probe(dev, ret, "hardware init failed\n");

	regmap_read(chg->regmap, SM5714_CHG_STATUS1, &chg->last_st1);
	regmap_read(chg->regmap, SM5714_CHG_STATUS2, &chg->last_st2);

	ret = devm_delayed_work_autocancel(dev, &chg->poll_work, sm5714_poll_work);
	if (ret)
		return ret;
	schedule_delayed_work(&chg->poll_work, msecs_to_jiffies(SM5714_POLL_MS));

	return 0;
}

static const struct of_device_id sm5714_charger_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_charger_of_match);

static const struct i2c_device_id sm5714_charger_id[] = {
	{ "sm5714-charger" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5714_charger_id);

static struct i2c_driver sm5714_charger_driver = {
	.driver = {
		.name = "sm5714-charger",
		.of_match_table = sm5714_charger_of_match,
	},
	.probe = sm5714_charger_probe,
	.id_table = sm5714_charger_id,
};
module_i2c_driver(sm5714_charger_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 charger driver");
MODULE_LICENSE("GPL");