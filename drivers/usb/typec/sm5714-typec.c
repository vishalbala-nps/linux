// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 USB Type-C port controller
 *
 * The SM5714 Type-C block does CC detection (DRP toggling, attach and
 * detach, Rp level and orientation) in hardware and reports the result in
 * CC_STATUS. This driver reports that to the Type-C class, switches the USB
 * controller between host and device, drives VBUS through a regulator
 * (normally the SM5714 charger's OTG boost) and publishes the Rp current
 * advertisement through a power supply for the charger.
 *
 * USB Power Delivery is not supported; the block's PD engine is left idle.
 *
 * Register layout and the initialisation sequence come from Samsung's
 * downstream SM5714 PDIC driver. The meaning of the CC_CNTL1 mode values is
 * inferred from how that driver uses them.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_wakeirq.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/role.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>

#define SM5714_TC_INT1			0x01	/* INT1..INT5: clear on read */
#define SM5714_TC_INT5			0x05
#define SM5714_TC_INT_MASK1		0x06	/* INT_MASK1..INT_MASK5 */
#define SM5714_TC_STATUS1		0x0b	/* STATUS1..STATUS5 */
#define  SM5714_TC_ST1_VBUSPOK		BIT(0)
#define  SM5714_TC_ST1_ATTACH		BIT(3)
#define  SM5714_TC_ST1_DETACH		BIT(4)
#define  SM5714_TC_ST2_SRC_ADV_CHG	BIT(4)	/* partner's Rp changed */
#define  SM5714_TC_ST5_CC_ABNORMAL	BIT(7)
#define SM5714_TC_JIGON_CONTROL		0x17
#define SM5714_TC_SYS_CNTL		0x1b
#define  SM5714_TC_SYS_SOFT_RESET	0x80
#define SM5714_TC_COMP_CNTL		0x1c
#define SM5714_TC_CLK_CNTL		0x1d
#define SM5714_TC_CORR_CNTL4		0x23
#define SM5714_TC_CORR_CNTL5		0x24
#define SM5714_TC_CC_STATUS		0x28
#define  SM5714_TC_CC_ATTACH_TYPE	GENMASK(2, 0)
#define  SM5714_TC_CC_ADV_CURR		GENMASK(4, 3)
#define  SM5714_TC_CC_FLIP		BIT(5)
#define SM5714_TC_CC_CNTL1		0x29
#define  SM5714_TC_CC_MODE_DRP		0x41	/* dual role, default Rp */
#define SM5714_TC_CC_CNTL3		0x2b
#define  SM5714_TC_CC_OP_EN		0x80
#define  SM5714_TC_CC_UNATT_SNK		0x82
#define SM5714_TC_CC_CNTL7		0x2f
#define  SM5714_TC_CC_CNTL7_ROLE	GENMASK(1, 0)
#define SM5714_TC_PD_CNTL1		0x38
#define SM5714_TC_PD_CNTL2		0x39
#define  SM5714_TC_PD_CNTL2_DFP		BIT(0)
#define  SM5714_TC_PD_CNTL2_SRC		BIT(1)
#define  SM5714_TC_PD_CNTL2_DETACHED	0x04
#define SM5714_TC_PD_CNTL4		0x3b
#define  SM5714_TC_PD_RESET_DONE	0x01
#define SM5714_TC_PD_STATE0		0xd5
#define  SM5714_TC_PD_STATE0_CC		GENMASK(3, 0)
#define  SM5714_TC_CC_ST_DISABLED	0x1
#define  SM5714_TC_CC_ST_ATTWAIT_SRC	0xa
#define SM5714_TC_PD_STATE3		0xd8
#define  SM5714_TC_PD_STATE3_RESET	GENMASK(2, 1)
#define SM5714_TC_MAX_REG		0xda

#define SM5714_TC_NUM_INT		5

/* CC_STATUS attach types */
enum sm5714_tc_attach {
	SM5714_TC_ATT_NONE = 0,
	SM5714_TC_ATT_SRC_PARTNER = 1,	/* charger/host attached: we sink */
	SM5714_TC_ATT_SNK_PARTNER = 2,	/* device attached: we source */
	SM5714_TC_ATT_AUDIO = 3,
	SM5714_TC_ATT_DEBUG_UNORI = 4,
	SM5714_TC_ATT_DEBUG_ORI = 5,
};

struct sm5714_tc {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;		/* serialises IRQ handling and teardown */

	struct fwnode_handle *connector;
	struct typec_capability cap;
	struct typec_port *port;
	struct typec_partner *partner;
	struct usb_role_switch *role_sw;
	struct regulator *vbus;
	bool vbus_on;

	struct power_supply *psy;
	int psy_online;			/* READ_ONCE/WRITE_ONCE */
	int psy_current_ua;

	enum sm5714_tc_attach state;
};

/* ---- hardware helpers --------------------------------------------------- */

/* Mask all interrupts and clear anything pending */
static int sm5714_tc_irq_mask_all(struct sm5714_tc *tc)
{
	static const u8 mask[SM5714_TC_NUM_INT] = { 0xff, 0xff, 0xff, 0xff, 0xff };
	u8 junk[SM5714_TC_NUM_INT];
	int ret;

	ret = regmap_bulk_write(tc->regmap, SM5714_TC_INT_MASK1, mask, sizeof(mask));
	if (ret)
		return ret;

	return regmap_bulk_read(tc->regmap, SM5714_TC_INT1, junk, sizeof(junk));
}

static int sm5714_tc_irq_unmask(struct sm5714_tc *tc)
{
	u8 mask[SM5714_TC_NUM_INT] = { 0xff, 0xff, 0xff, 0xff, 0xff };

	mask[0] &= ~(SM5714_TC_ST1_VBUSPOK | SM5714_TC_ST1_ATTACH |
		     SM5714_TC_ST1_DETACH);
	mask[1] &= ~SM5714_TC_ST2_SRC_ADV_CHG;
	mask[4] &= ~SM5714_TC_ST5_CC_ABNORMAL;

	return regmap_bulk_write(tc->regmap, SM5714_TC_INT_MASK1, mask, sizeof(mask));
}

static int sm5714_tc_set_mode(struct sm5714_tc *tc)
{
	/*
	 * Only DRP is set explicitly. Downstream leaves CC_CNTL1 untouched in
	 * off-mode charging, where the chip must act as a plain sink, so the
	 * reset default is used for sink-only ports.
	 */
	if (tc->cap.type == TYPEC_PORT_DRP)
		return regmap_write(tc->regmap, SM5714_TC_CC_CNTL1,
				    SM5714_TC_CC_MODE_DRP);

	return 0;
}

/* Release the PD engine's reset wait so it does not hold up the CC logic */
static int sm5714_tc_pd_reset_done(struct sm5714_tc *tc)
{
	unsigned int val;
	int ret;

	ret = regmap_read(tc->regmap, SM5714_TC_PD_STATE3, &val);
	if (ret)
		return ret;

	return regmap_write(tc->regmap, SM5714_TC_PD_CNTL4,
			    (val & SM5714_TC_PD_STATE3_RESET) ?
			    SM5714_TC_PD_RESET_DONE : 0);
}

/*
 * Kick the CC state machine when nothing is attached, as downstream does:
 * from CC_DISABLED (or stuck in ATTWAIT_SRC with VBUS present) go to
 * unattached sink, otherwise just (re-)enable CC operation.
 */
static int sm5714_tc_kick_cc(struct sm5714_tc *tc)
{
	unsigned int st1, st0, cc;
	int ret;

	ret = regmap_read(tc->regmap, SM5714_TC_STATUS1, &st1);
	if (ret)
		return ret;
	if (st1 & SM5714_TC_ST1_ATTACH)
		return 0;

	ret = regmap_read(tc->regmap, SM5714_TC_PD_STATE0, &st0);
	if (ret)
		return ret;

	cc = FIELD_GET(SM5714_TC_PD_STATE0_CC, st0);
	if (cc == SM5714_TC_CC_ST_DISABLED ||
	    (cc == SM5714_TC_CC_ST_ATTWAIT_SRC && (st1 & SM5714_TC_ST1_VBUSPOK)))
		return regmap_write(tc->regmap, SM5714_TC_CC_CNTL3,
				    SM5714_TC_CC_UNATT_SNK);

	return regmap_write(tc->regmap, SM5714_TC_CC_CNTL3, SM5714_TC_CC_OP_EN);
}

/* Downstream soft-resets the block if these hold unexpected values */
static bool sm5714_tc_sane(struct sm5714_tc *tc)
{
	unsigned int jig, comp, clk;

	if (regmap_read(tc->regmap, SM5714_TC_JIGON_CONTROL, &jig) ||
	    regmap_read(tc->regmap, SM5714_TC_COMP_CNTL, &comp) ||
	    regmap_read(tc->regmap, SM5714_TC_CLK_CNTL, &clk))
		return false;

	return (jig == 0x1 || jig == 0x3) && comp == 0x98 && clk == 0x08;
}

static int sm5714_tc_hw_init(struct sm5714_tc *tc)
{
	int ret;

	if (!sm5714_tc_sane(tc)) {
		dev_info(tc->dev, "unexpected state, soft-resetting\n");
		ret = regmap_write(tc->regmap, SM5714_TC_SYS_CNTL,
				   SM5714_TC_SYS_SOFT_RESET);
		if (ret)
			return ret;
		msleep(20);
	}

	ret = sm5714_tc_irq_mask_all(tc);
	if (ret)
		return ret;

	ret = sm5714_tc_set_mode(tc);
	if (ret)
		return ret;

	/* No SBU sourcing, no low-power water detection */
	ret = regmap_write(tc->regmap, SM5714_TC_CORR_CNTL5, 0);
	if (ret)
		return ret;
	ret = regmap_write(tc->regmap, SM5714_TC_CORR_CNTL4, 0);
	if (ret)
		return ret;

	ret = sm5714_tc_pd_reset_done(tc);
	if (ret)
		return ret;

	return sm5714_tc_kick_cc(tc);
}

/* Return the block to its unattached configuration (downstream detach) */
static void sm5714_tc_hw_detach(struct sm5714_tc *tc)
{
	regmap_write(tc->regmap, SM5714_TC_CORR_CNTL5, 0);
	regmap_write(tc->regmap, SM5714_TC_PD_CNTL2, SM5714_TC_PD_CNTL2_DETACHED);
	sm5714_tc_pd_reset_done(tc);
	regmap_write(tc->regmap, SM5714_TC_PD_CNTL1, 0);
	regmap_clear_bits(tc->regmap, SM5714_TC_CC_CNTL7, SM5714_TC_CC_CNTL7_ROLE);
	sm5714_tc_set_mode(tc);
	regmap_write(tc->regmap, SM5714_TC_CC_CNTL3, SM5714_TC_CC_OP_EN);
}

/* ---- state handling ----------------------------------------------------- */

static void sm5714_tc_set_vbus(struct sm5714_tc *tc, bool on)
{
	int ret;

	if (!tc->vbus || tc->vbus_on == on)
		return;

	ret = on ? regulator_enable(tc->vbus) : regulator_disable(tc->vbus);
	if (ret) {
		dev_err(tc->dev, "failed to switch VBUS %s: %d\n",
			on ? "on" : "off", ret);
		return;
	}
	tc->vbus_on = on;
}

static void sm5714_tc_set_role(struct sm5714_tc *tc, enum usb_role role)
{
	int ret;

	if (!tc->role_sw)
		return;

	ret = usb_role_switch_set_role(tc->role_sw, role);
	if (ret)
		dev_err(tc->dev, "failed to set USB role %d: %d\n", role, ret);
}

static void sm5714_tc_set_psy(struct sm5714_tc *tc, int online, int current_ua)
{
	if (READ_ONCE(tc->psy_online) == online &&
	    READ_ONCE(tc->psy_current_ua) == current_ua)
		return;

	WRITE_ONCE(tc->psy_online, online);
	WRITE_ONCE(tc->psy_current_ua, current_ua);
	power_supply_changed(tc->psy);
}

static void sm5714_tc_update_current(struct sm5714_tc *tc, unsigned int cc)
{
	enum typec_pwr_opmode mode;
	int ua;

	switch (FIELD_GET(SM5714_TC_CC_ADV_CURR, cc)) {
	case 0:
		mode = TYPEC_PWR_MODE_USB;
		ua = 500000;
		break;
	case 1:
		mode = TYPEC_PWR_MODE_1_5A;
		ua = 1500000;
		break;
	default:
		mode = TYPEC_PWR_MODE_3_0A;
		ua = 3000000;
		break;
	}

	typec_set_pwr_opmode(tc->port, mode);
	sm5714_tc_set_psy(tc, 1, ua);
}

/*
 * Tell the mode switches linked to the connector (for example a D+/D-
 * switch) what the port is doing. Without any, this does nothing.
 */
static void sm5714_tc_set_mux(struct sm5714_tc *tc, int mode)
{
	int ret = typec_set_mode(tc->port, mode);

	if (ret)
		dev_warn(tc->dev, "failed to set mux mode %d: %d\n", mode, ret);
}

static void sm5714_tc_detach(struct sm5714_tc *tc)
{
	sm5714_tc_set_role(tc, USB_ROLE_NONE);
	sm5714_tc_set_mux(tc, TYPEC_STATE_SAFE);
	sm5714_tc_set_vbus(tc, false);
	sm5714_tc_set_psy(tc, 0, 0);

	if (tc->partner) {
		typec_unregister_partner(tc->partner);
		tc->partner = NULL;
	}

	typec_set_orientation(tc->port, TYPEC_ORIENTATION_NONE);
	typec_set_pwr_opmode(tc->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(tc->port, TYPEC_SINK);
	typec_set_vconn_role(tc->port, TYPEC_SINK);
	typec_set_data_role(tc->port, TYPEC_DEVICE);

	sm5714_tc_hw_detach(tc);
	dev_dbg(tc->dev, "detached\n");
}

static void sm5714_tc_attach(struct sm5714_tc *tc, enum sm5714_tc_attach type,
			     unsigned int cc)
{
	struct typec_partner_desc desc = { .accessory = TYPEC_ACCESSORY_NONE };
	struct typec_partner *partner;

	typec_set_orientation(tc->port, (cc & SM5714_TC_CC_FLIP) ?
			      TYPEC_ORIENTATION_REVERSE :
			      TYPEC_ORIENTATION_NORMAL);

	switch (type) {
	case SM5714_TC_ATT_SRC_PARTNER:
		regmap_clear_bits(tc->regmap, SM5714_TC_PD_CNTL2,
				  SM5714_TC_PD_CNTL2_DFP | SM5714_TC_PD_CNTL2_SRC);
		typec_set_pwr_role(tc->port, TYPEC_SINK);
		typec_set_data_role(tc->port, TYPEC_DEVICE);
		sm5714_tc_set_mux(tc, TYPEC_STATE_USB);
		sm5714_tc_set_role(tc, USB_ROLE_DEVICE);
		sm5714_tc_update_current(tc, cc);
		break;

	case SM5714_TC_ATT_SNK_PARTNER:
		sm5714_tc_set_vbus(tc, true);
		regmap_set_bits(tc->regmap, SM5714_TC_PD_CNTL2,
				SM5714_TC_PD_CNTL2_DFP | SM5714_TC_PD_CNTL2_SRC);
		typec_set_pwr_role(tc->port, TYPEC_SOURCE);
		typec_set_data_role(tc->port, TYPEC_HOST);
		sm5714_tc_set_mux(tc, TYPEC_STATE_USB);
		sm5714_tc_set_role(tc, USB_ROLE_HOST);
		break;

	case SM5714_TC_ATT_AUDIO:
		desc.accessory = TYPEC_ACCESSORY_AUDIO;
		sm5714_tc_set_mux(tc, TYPEC_MODE_AUDIO);
		break;

	case SM5714_TC_ATT_DEBUG_UNORI:
	case SM5714_TC_ATT_DEBUG_ORI:
		desc.accessory = TYPEC_ACCESSORY_DEBUG;
		sm5714_tc_set_mux(tc, TYPEC_MODE_DEBUG);
		break;

	default:
		break;
	}

	partner = typec_register_partner(tc->port, &desc);
	if (IS_ERR(partner))
		dev_err(tc->dev, "failed to register partner: %ld\n",
			PTR_ERR(partner));
	else
		tc->partner = partner;

	dev_dbg(tc->dev, "attached, type %d, CC_STATUS 0x%02x\n", type, cc);
}

/*
 * Bring the software state in line with the hardware. State based rather
 * than edge based, so a missed interrupt is caught on the next one.
 * Caller holds tc->lock.
 */
static void sm5714_tc_update(struct sm5714_tc *tc)
{
	enum sm5714_tc_attach type = SM5714_TC_ATT_NONE;
	unsigned int st1, cc;

	if (regmap_read(tc->regmap, SM5714_TC_STATUS1, &st1) ||
	    regmap_read(tc->regmap, SM5714_TC_CC_STATUS, &cc))
		return;

	if (st1 & SM5714_TC_ST1_ATTACH)
		type = FIELD_GET(SM5714_TC_CC_ATTACH_TYPE, cc);

	if (type == tc->state) {
		/* Same partner: the source may have changed its Rp */
		if (type == SM5714_TC_ATT_SRC_PARTNER)
			sm5714_tc_update_current(tc, cc);
		else if (type == SM5714_TC_ATT_NONE)
			sm5714_tc_kick_cc(tc);
		return;
	}

	if (tc->state != SM5714_TC_ATT_NONE)
		sm5714_tc_detach(tc);

	tc->state = type;

	if (type != SM5714_TC_ATT_NONE)
		sm5714_tc_attach(tc, type, cc);
}

static irqreturn_t sm5714_tc_irq(int irq, void *data)
{
	struct sm5714_tc *tc = data;
	u8 intr[SM5714_TC_NUM_INT];
	int ret, i;
	bool any = false;

	mutex_lock(&tc->lock);

	/* Reading INT1..INT5 clears them and releases the level IRQ */
	ret = regmap_bulk_read(tc->regmap, SM5714_TC_INT1, intr, sizeof(intr));
	if (ret) {
		mutex_unlock(&tc->lock);
		return IRQ_NONE;
	}

	for (i = 0; i < SM5714_TC_NUM_INT; i++)
		any |= intr[i];

	if (any) {
		dev_dbg(tc->dev, "INT %*ph\n", SM5714_TC_NUM_INT, intr);
		if (intr[4] & SM5714_TC_ST5_CC_ABNORMAL)
			dev_warn(tc->dev, "CC line fault (CC shorted to VBUS?)\n");
		sm5714_tc_update(tc);
	}

	mutex_unlock(&tc->lock);

	return any ? IRQ_HANDLED : IRQ_NONE;
}

/* ---- power supply: the partner's current advertisement ------------------ */

static int sm5714_tc_psy_get_property(struct power_supply *psy,
				      enum power_supply_property psp,
				      union power_supply_propval *val)
{
	struct sm5714_tc *tc = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = READ_ONCE(tc->psy_online);
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = READ_ONCE(tc->psy_current_ua);
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property sm5714_tc_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc sm5714_tc_psy_desc = {
	.name = "sm5714-typec",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm5714_tc_psy_props,
	.num_properties = ARRAY_SIZE(sm5714_tc_psy_props),
	.get_property = sm5714_tc_psy_get_property,
};

/*
 * The SM5714 charger takes its input current limit from this supply. Name it
 * as the supplicant here instead of listing this node in the charger's
 * power-supplies: that would make the charger wait for this driver, while
 * this driver waits for the charger's OTG regulator (vbus-supply).
 */
static char *sm5714_tc_supplied_to[] = { "sm5714-charger" };

/* ---- probe / teardown --------------------------------------------------- */

static bool sm5714_tc_precious_reg(struct device *dev, unsigned int reg)
{
	return reg >= SM5714_TC_INT1 && reg <= SM5714_TC_INT5;
}

static const struct regmap_config sm5714_tc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SM5714_TC_MAX_REG,
	.precious_reg = sm5714_tc_precious_reg,
};

static void sm5714_tc_put_fwnode(void *data)
{
	fwnode_handle_put(data);
}

static void sm5714_tc_put_role_sw(void *data)
{
	usb_role_switch_put(data);
}

static void sm5714_tc_unregister_port(void *data)
{
	typec_unregister_port(data);
}

static void sm5714_tc_disable_wakeup(void *data)
{
	struct device *dev = data;

	dev_pm_clear_wake_irq(dev);
	device_init_wakeup(dev, false);
}

/* Runs on unbind after the IRQ is freed: drop VBUS and the USB role */
static void sm5714_tc_teardown(void *data)
{
	struct sm5714_tc *tc = data;

	mutex_lock(&tc->lock);
	if (tc->state != SM5714_TC_ATT_NONE)
		sm5714_tc_detach(tc);
	tc->state = SM5714_TC_ATT_NONE;
	mutex_unlock(&tc->lock);
}

static int sm5714_tc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct sm5714_tc *tc;
	int ret;

	tc = devm_kzalloc(dev, sizeof(*tc), GFP_KERNEL);
	if (!tc)
		return -ENOMEM;

	tc->dev = dev;
	ret = devm_mutex_init(dev, &tc->lock);
	if (ret)
		return ret;

	tc->regmap = devm_regmap_init_i2c(client, &sm5714_tc_regmap_config);
	if (IS_ERR(tc->regmap))
		return dev_err_probe(dev, PTR_ERR(tc->regmap), "regmap init failed\n");

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "no interrupt\n");

	/* Port capabilities from the usb-c-connector child node */
	tc->connector = device_get_named_child_node(dev, "connector");
	if (!tc->connector)
		return dev_err_probe(dev, -ENODEV, "no connector node\n");
	ret = devm_add_action_or_reset(dev, sm5714_tc_put_fwnode, tc->connector);
	if (ret)
		return ret;

	ret = typec_get_fw_cap(&tc->cap, tc->connector);
	if (ret)
		return dev_err_probe(dev, ret, "invalid connector properties\n");
	if (tc->cap.type == TYPEC_PORT_SRC)
		return dev_err_probe(dev, -EINVAL, "source-only ports are not supported\n");
	tc->cap.revision = USB_TYPEC_REV_1_2;
	tc->cap.driver_data = tc;

	/* VBUS for host mode (optional: sink-only boards have none) */
	tc->vbus = devm_regulator_get_optional(dev, "vbus");
	if (IS_ERR(tc->vbus)) {
		if (PTR_ERR(tc->vbus) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(tc->vbus), "failed to get VBUS\n");
		tc->vbus = NULL;
	}

	/* USB controller role switch (optional) */
	tc->role_sw = fwnode_usb_role_switch_get(tc->connector);
	if (IS_ERR(tc->role_sw))
		return dev_err_probe(dev, PTR_ERR(tc->role_sw), "failed to get role switch\n");
	if (tc->role_sw) {
		ret = devm_add_action_or_reset(dev, sm5714_tc_put_role_sw, tc->role_sw);
		if (ret)
			return ret;
	}

	ret = sm5714_tc_hw_init(tc);
	if (ret)
		return dev_err_probe(dev, ret, "hardware init failed\n");

	tc->port = typec_register_port(dev, &tc->cap);
	if (IS_ERR(tc->port))
		return dev_err_probe(dev, PTR_ERR(tc->port), "failed to register port\n");
	ret = devm_add_action_or_reset(dev, sm5714_tc_unregister_port, tc->port);
	if (ret)
		return ret;

	psy_cfg.drv_data = tc;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy_cfg.supplied_to = sm5714_tc_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(sm5714_tc_supplied_to);
	tc->psy = devm_power_supply_register(dev, &sm5714_tc_psy_desc, &psy_cfg);
	if (IS_ERR(tc->psy))
		return dev_err_probe(dev, PTR_ERR(tc->psy), "failed to register power supply\n");

	/* Registered before the IRQ, so it runs after the IRQ is freed */
	ret = devm_add_action_or_reset(dev, sm5714_tc_teardown, tc);
	if (ret)
		return ret;

	/* Trigger type (level low) comes from DT */
	ret = devm_request_threaded_irq(dev, client->irq, NULL, sm5714_tc_irq,
					IRQF_ONESHOT, "sm5714-typec", tc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	/* Pick up whatever is already plugged in, then enable interrupts */
	mutex_lock(&tc->lock);
	sm5714_tc_update(tc);
	ret = sm5714_tc_irq_unmask(tc);
	mutex_unlock(&tc->lock);
	if (ret)
		return dev_err_probe(dev, ret, "failed to unmask interrupts\n");

	/* Plugging something in wakes the system */
	device_init_wakeup(dev, true);
	ret = dev_pm_set_wake_irq(dev, client->irq);
	if (ret)
		dev_warn(dev, "failed to set wake IRQ: %d\n", ret);
	ret = devm_add_action_or_reset(dev, sm5714_tc_disable_wakeup, dev);
	if (ret)
		return ret;

	dev_info(dev, "SM5714 Type-C port, %s\n",
		 tc->cap.type == TYPEC_PORT_DRP ? "dual role" : "sink only");

	return 0;
}

static const struct of_device_id sm5714_tc_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-typec" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_tc_of_match);

static const struct i2c_device_id sm5714_tc_id[] = {
	{ "sm5714-typec" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5714_tc_id);

static struct i2c_driver sm5714_tc_driver = {
	.driver = {
		.name = "sm5714-typec",
		.of_match_table = sm5714_tc_of_match,
	},
	.probe = sm5714_tc_probe,
	.id_table = sm5714_tc_id,
};
module_i2c_driver(sm5714_tc_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 USB Type-C port controller driver");
MODULE_LICENSE("GPL");