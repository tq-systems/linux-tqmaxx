// SPDX-License-Identifier: GPL-2.0
/*
 * Vision Components MIPI Module Controller
 *
 * Copyright 2021 Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#define VC_MIPI_REG_RESET			0x0100
#define VC_MIPI_REG_RESET_POWER_DOWN		BIT(1)
#define VC_MIPI_REG_RESET_RESET			BIT(0)
#define VC_MIPI_REG_STATUS			0x0101
#define VC_MIPI_REG_STATUS_OFF			0x00
#define VC_MIPI_REG_STATUS_ERROR		0x01
#define VC_MIPI_REG_STATUS_ON			0x80
#define VC_MIPI_REG_MODE			0x0102
#define VC_MIPI_REG_IO				0x0103
#define VC_MIPI_REG_MODULE_ADDR			0x0104
#define VC_MIPI_REG_SENSOR_ADDR			0x0105
#define VC_MIPI_REG_OUTPUT_OVERRIDE		0x0106
#define VC_MIPI_REG_INPUT			0x0107

#define VC_MIPI_REG_ROM				0x1000

struct vc_mipi_ctrl {
	struct device *dev;
	struct regmap *regmap;
	struct regulator *supply;
	struct clk_hw *clk_hw;
};

/* -----------------------------------------------------------------------------
 * Regulator
 */

static int vc_mipi_regulator_enable(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_write(rdev->regmap, VC_MIPI_REG_RESET, 0);
	if (ret < 0)
		return ret;

	msleep(500);

	ret = regmap_read(rdev->regmap, VC_MIPI_REG_STATUS, &val);
	if (ret < 0)
		return ret;

	if (val != VC_MIPI_REG_STATUS_ON) {
		dev_err(&rdev->dev, "Sensor failed to initialize (0x%02x)\n",
			val);
		return -EIO;
	}

	return 0;
}

static int vc_mipi_regulator_disable(struct regulator_dev *rdev)
{
	return regmap_write(rdev->regmap, VC_MIPI_REG_RESET,
			    VC_MIPI_REG_RESET_POWER_DOWN |
			    VC_MIPI_REG_RESET_RESET);
}

static int vc_mipi_regulator_is_enabled(struct regulator_dev *rdev)
{
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, VC_MIPI_REG_RESET, &val);
	if (ret < 0)
		return ret;

	return !(val & VC_MIPI_REG_RESET_POWER_DOWN);
}

static const struct regulator_ops vc_mipi_regulator_ops = {
	.enable = vc_mipi_regulator_enable,
	.disable = vc_mipi_regulator_disable,
	.is_enabled = vc_mipi_regulator_is_enabled,
};

static const struct regulator_desc vc_mipi_regulator = {
	.name = "vc-mipi",
	.ops = &vc_mipi_regulator_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
};

static int vc_mipi_regulator_init(struct vc_mipi_ctrl *ctrl)
{
	struct regulator_config config = { };
	struct regulator_dev *rdev;

	config.dev = ctrl->dev;
	config.of_node = ctrl->dev->of_node;
	config.regmap = ctrl->regmap;

	rdev = devm_regulator_register(ctrl->dev, &vc_mipi_regulator, &config);
	if (IS_ERR(rdev)) {
		return PTR_ERR(rdev);
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Clock
 */

static int vc_mipi_clk_init(struct vc_mipi_ctrl *ctrl)
{
	u32 freq;
	int ret;

	ret = of_property_read_u32(ctrl->dev->of_node, "clock-frequency",
				   &freq);
	if (ret < 0) {
		dev_err(ctrl->dev, "Failed to retrieve clock frequency: %d\n",
			ret);
		return ret;
	}

	ctrl->clk_hw = clk_hw_register_fixed_rate(ctrl->dev, "clk", NULL, 0,
						  freq);
	if (IS_ERR(ctrl->clk_hw))
		return PTR_ERR(ctrl->clk_hw);

	ret = devm_of_clk_add_hw_provider(ctrl->dev, of_clk_hw_simple_get,
					  ctrl->clk_hw);
	if (ret < 0) {
		clk_hw_unregister_fixed_rate(ctrl->clk_hw);
		return ret;
	}

	return 0;
}

static void vc_mipi_clk_cleanup(struct vc_mipi_ctrl *ctrl)
{
	clk_hw_unregister_fixed_rate(ctrl->clk_hw);
}

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static const struct regmap_config vc_mipi_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static int vc_mipi_i2c_probe(struct i2c_client *i2c)
{
	struct vc_mipi_ctrl *ctrl;
	char data[12];
	int ret;

	ctrl = devm_kzalloc(&i2c->dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->dev = &i2c->dev;

	i2c_set_clientdata(i2c, ctrl);

	ctrl->supply = devm_regulator_get(ctrl->dev, "vcc");
	if (IS_ERR(ctrl->supply)) {
		ret = PTR_ERR(ctrl->supply);
		dev_err(ctrl->dev, "Failed to get vcc supply: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(ctrl->supply);
	if (ret < 0) {
		dev_err(ctrl->dev, "Failed to enable vcc supply: %d\n", ret);
		return ret;
	}

	ctrl->regmap = devm_regmap_init_i2c(i2c, &vc_mipi_regmap_config);
	if (IS_ERR(ctrl->regmap)) {
		ret = PTR_ERR(ctrl->regmap);
		dev_err(ctrl->dev, "Failed to init regmap: %d\n", ret);
		goto error;
	}

	ret = regmap_raw_read(ctrl->regmap, VC_MIPI_REG_ROM, data,
			      sizeof(data));
	if (ret < 0) {
		dev_err(ctrl->dev, "Failed to read ROM: %d\n", ret);
		goto error;
	}

	if (memcmp(data, "mmipi-module", sizeof(data))) {
		dev_err(ctrl->dev, "Invalid ROM magic value\n");
		print_hex_dump(KERN_INFO, "rom: ", DUMP_PREFIX_OFFSET, 16, 1,
			       data, sizeof(data), true);
		ret = -EINVAL;
		goto error;
	}

	ret = vc_mipi_regulator_init(ctrl);
	if (ret < 0) {
		dev_err(ctrl->dev, "Failed to register regulator\n");
		goto error;
	}

	ret = vc_mipi_clk_init(ctrl);
	if (ret < 0) {
		dev_err(ctrl->dev, "Failed to register clock\n");
		goto error;
	}

	return 0;

error:
	regulator_disable(ctrl->supply);
	return ret;
}

static int vc_mipi_i2c_remove(struct i2c_client *i2c)
{
	struct vc_mipi_ctrl *ctrl = i2c_get_clientdata(i2c);

	vc_mipi_clk_cleanup(ctrl);
	regulator_disable(ctrl->supply);

	return 0;
}

static const struct of_device_id vc_mipi_dt_ids[] = {
	{ .compatible = "vision-components,mipi-module" },
	{},
};
MODULE_DEVICE_TABLE(of, vc_mipi_dt_ids);

static struct i2c_driver vc_mipi_driver = {
	.driver = {
		.name = "vc-mipi",
		.of_match_table = of_match_ptr(vc_mipi_dt_ids),
	},
	.probe_new = vc_mipi_i2c_probe,
	.remove = vc_mipi_i2c_remove,
};

module_i2c_driver(vc_mipi_driver);

MODULE_AUTHOR("Laurent Pinchart <laurent.pinchart@ideasonboard.com>");
MODULE_DESCRIPTION("Driver for the Vision Components MIPI Module Controller");
MODULE_LICENSE("GPL v2");
