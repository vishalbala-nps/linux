// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5714 MUIC as a USB Type-C mode switch
 *
 * The SM5714 MUIC is an analog switch that routes the connector's D+/D-
 * lines to the SoC's USB PHY, a UART, JTAG, or nowhere. Left in its
 * automatic mode it only connects USB when it detects an incoming VBUS
 * (a charger or a USB host), so USB 2.0 data does not work while the port
 * supplies VBUS itself as a host.
 *
 * This driver registers the switch as a Type-C mux. The port driver reports
 * the connector state through typec_set_mode() and the switch follows it:
 * USB (and any alternate mode, which leaves D+/D- for USB 2.0) connects
 * D+/D- to the USB PHY, an audio accessory opens them, and the safe state
 * (nothing attached) hands the switch back to the chip's automatic mode.
 *
 * The MUIC's own detection (BC1.2, AFC/QC, ID resistor) is not supported:
 * its interrupts are masked so they cannot hold the SM5714's shared
 * interrupt line.
 *
 * Register layout from Samsung's downstream SM5714 MUIC driver.
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_mux.h>

#define SM5714_MUIC_DEVICE_ID		0x00
#define  SM5714_MUIC_ID_SM5714		0x01
#define SM5714_MUIC_INT1		0x01	/* INT1..INT2: clear on read */
#define SM5714_MUIC_INT2		0x02
#define SM5714_MUIC_INTMASK1		0x03
#define SM5714_MUIC_INTMASK2		0x04
#define SM5714_MUIC_CNTL		0x05
#define  SM5714_MUIC_CNTL_NSW_OPEN	BIT(5)	/* 0 forces the switch open */
#define SM5714_MUIC_MANUAL_SW		0x06
#define  SM5714_MUIC_SW_MANUAL		BIT(7)	/* 0: the chip decides */
#define  SM5714_MUIC_SW_DM		GENMASK(5, 3)
#define  SM5714_MUIC_SW_DP		GENMASK(2, 0)
#define SM5714_MUIC_MAX_REG		0x1e

/* D+/D- path values for MANUAL_SW */
#define SM5714_MUIC_PATH_OPEN		0x0
#define SM5714_MUIC_PATH_USB		0x1

#define SM5714_MUIC_SW_MASK	(SM5714_MUIC_SW_MANUAL | SM5714_MUIC_SW_DM | \
				 SM5714_MUIC_SW_DP)

struct sm5714_muic {
	struct device *dev;
	struct regmap *regmap;
	struct typec_mux_dev *mux;
};

static bool sm5714_muic_precious_reg(struct device *dev, unsigned int reg)
{
	return reg == SM5714_MUIC_INT1 || reg == SM5714_MUIC_INT2;
}

static const struct regmap_config sm5714_muic_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SM5714_MUIC_MAX_REG,
	.precious_reg = sm5714_muic_precious_reg,
};

/* Manual mode, both lines on @path. Bit 6 is left alone, as downstream does */
static int sm5714_muic_set_path(struct sm5714_muic *muic, unsigned int path)
{
	return regmap_update_bits(muic->regmap, SM5714_MUIC_MANUAL_SW,
				  SM5714_MUIC_SW_MASK,
				  SM5714_MUIC_SW_MANUAL |
				  FIELD_PREP(SM5714_MUIC_SW_DM, path) |
				  FIELD_PREP(SM5714_MUIC_SW_DP, path));
}

static int sm5714_muic_set_auto(struct sm5714_muic *muic)
{
	return regmap_clear_bits(muic->regmap, SM5714_MUIC_MANUAL_SW,
				 SM5714_MUIC_SW_MANUAL);
}

static int sm5714_muic_mux_set(struct typec_mux_dev *mux,
			       struct typec_mux_state *state)
{
	struct sm5714_muic *muic = typec_mux_get_drvdata(mux);
	int ret;

	/*
	 * With state->alt set, mode is the alternate mode's own state, and
	 * alternate modes never take over D+/D-, so USB 2.0 stays connected.
	 */
	if (state->alt)
		ret = sm5714_muic_set_path(muic, SM5714_MUIC_PATH_USB);
	else if (state->mode == TYPEC_STATE_SAFE)
		ret = sm5714_muic_set_auto(muic);
	else if (state->mode == TYPEC_MODE_AUDIO)
		ret = sm5714_muic_set_path(muic, SM5714_MUIC_PATH_OPEN);
	else
		ret = sm5714_muic_set_path(muic, SM5714_MUIC_PATH_USB);

	if (ret)
		dev_err(muic->dev, "failed to set mode %lu: %d\n", state->mode, ret);
	else
		dev_dbg(muic->dev, "mode %lu%s\n", state->mode,
			state->alt ? " (alt)" : "");

	return ret;
}

static void sm5714_muic_unregister_mux(void *data)
{
	typec_mux_unregister(data);
}

static void sm5714_muic_restore_auto(void *data)
{
	sm5714_muic_set_auto(data);
}

static int sm5714_muic_probe(struct i2c_client *client)
{
	static const u8 mask_all[] = { 0xff, 0xff };
	struct typec_mux_desc mux_desc = { };
	struct device *dev = &client->dev;
	struct sm5714_muic *muic;
	unsigned int id;
	u8 junk[2];
	int ret;

	muic = devm_kzalloc(dev, sizeof(*muic), GFP_KERNEL);
	if (!muic)
		return -ENOMEM;
	muic->dev = dev;

	muic->regmap = devm_regmap_init_i2c(client, &sm5714_muic_regmap_config);
	if (IS_ERR(muic->regmap))
		return dev_err_probe(dev, PTR_ERR(muic->regmap),
				     "regmap init failed\n");

	ret = regmap_read(muic->regmap, SM5714_MUIC_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device id\n");
	if (id != SM5714_MUIC_ID_SM5714)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected device id 0x%02x\n", id);

	/* Nothing handles MUIC events: keep them off the shared IRQ line */
	ret = regmap_bulk_write(muic->regmap, SM5714_MUIC_INTMASK1, mask_all,
				sizeof(mask_all));
	if (ret)
		return dev_err_probe(dev, ret, "failed to mask interrupts\n");
	ret = regmap_bulk_read(muic->regmap, SM5714_MUIC_INT1, junk, sizeof(junk));
	if (ret)
		return ret;

	/* Never force the switch open; start in automatic mode */
	ret = regmap_set_bits(muic->regmap, SM5714_MUIC_CNTL,
			      SM5714_MUIC_CNTL_NSW_OPEN);
	if (!ret)
		ret = sm5714_muic_set_auto(muic);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init switch\n");

	/* Registered first so it runs last, after the mux is gone */
	ret = devm_add_action_or_reset(dev, sm5714_muic_restore_auto, muic);
	if (ret)
		return ret;

	mux_desc.fwnode = dev_fwnode(dev);
	mux_desc.drvdata = muic;
	mux_desc.set = sm5714_muic_mux_set;
	muic->mux = typec_mux_register(dev, &mux_desc);
	if (IS_ERR(muic->mux))
		return dev_err_probe(dev, PTR_ERR(muic->mux),
				     "failed to register mode switch\n");

	return devm_add_action_or_reset(dev, sm5714_muic_unregister_mux,
					muic->mux);
}

static const struct of_device_id sm5714_muic_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-muic" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_muic_of_match);

static const struct i2c_device_id sm5714_muic_id[] = {
	{ "sm5714-muic" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5714_muic_id);

static struct i2c_driver sm5714_muic_driver = {
	.driver = {
		.name = "sm5714-muic",
		.of_match_table = sm5714_muic_of_match,
	},
	.probe = sm5714_muic_probe,
	.id_table = sm5714_muic_id,
};
module_i2c_driver(sm5714_muic_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5714 MUIC Type-C mode switch driver");
MODULE_LICENSE("GPL");