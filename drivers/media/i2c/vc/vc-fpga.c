// SPDX-License-Identifier: GPL-2.0

#include "vc-fpga.h"

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/media-bus-format.h>

#include <linux/gpio/driver.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/reset-controller.h>

#define R100_RESET		BIT(0)
#define R100_PWROFF		BIT(1)

struct vc_fpga {
	struct i2c_client		*i2c;
	struct mutex			mode_lock;
	struct vc_rom_table		rom_table;
	struct vc_rom_info		rom_info;

	uint8_t				cfg_num_code;
	bool				need_cfg;
	bool				is_reset;

	uint8_t				shadow_r100;

	struct regulator_dev		*rdev;
	struct reset_controller_dev	reset_ctrl;
	struct gpio_chip		gpioc;
};

#define gpioc_to_fpga(_chip)	container_of((_chip), struct vc_fpga, gpioc)
#define reset_to_fpga(_rdev)	container_of((_rdev), struct vc_fpga, reset_ctrl)

static int vc_fpga_write(struct vc_fpga *fpga, uint16_t addr, uint8_t data)
{
	uint8_t			tx[] = {
		[0] = (addr >> 8) & 0xff,
		[1] = (addr >> 0) & 0xff,
		[2] = data,
	};

	struct i2c_msg		msg[] = {
		[0] = {
			.addr	= fpga->i2c->addr,
			.buf	= tx,
			.len	= sizeof(tx),
		}
	};

	int			rc;

	rc = i2c_transfer(fpga->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (rc < 0)
		dev_warn(&fpga->i2c->dev, "failed to write [%04x] = %02x\n",
			 addr, data);

	return rc;
}

static int vc_fpga_read(struct vc_fpga *fpga, uint16_t addr, uint8_t *data)
{
	uint8_t			tx[] = {
		[0] = (addr >> 8) & 0xff,
		[1] = (addr >> 0) & 0xff,
	};

	uint8_t			rx[1];

	struct i2c_msg		msg[] = {
		[0]	= {
			.addr	= fpga->i2c->addr,
			.buf	= tx,
			.len	= sizeof(tx),
		},
		[1]	= {
			.addr	= fpga->i2c->addr,
			.flags = I2C_M_RD,
			.buf	= rx,
			.len	= sizeof(rx),
		},
	};

	int			rc;

	rc = i2c_transfer(fpga->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (rc < 0) {
		dev_warn(&fpga->i2c->dev, "failed to read 0x%04x: %d\n",
			 addr, rc);
		goto out;
	}

	if (data)
		*data = rx[0];

	rc = 0;

out:
	return rc;
}

enum vc_fpga_cfg_action vc_fpga_cfg(struct vc_fpga *fpga, uint8_t mode)
{
	enum vc_fpga_cfg_action		res = VC_FPGA_CFG_SAME;

	mutex_lock(&fpga->mode_lock);
	if (fpga->cfg_num_code != mode) {
		fpga->need_cfg = true;
		fpga->cfg_num_code = mode;
	}

	if (fpga->need_cfg || fpga->shadow_r100 != 0)
		res = VC_FPGA_CFG_NEED_RESET;
	mutex_unlock(&fpga->mode_lock);

	return res;
}
EXPORT_SYMBOL(vc_fpga_cfg);

static void vc_fpga_show_info(struct vc_fpga *fpga)
{
	struct device			*dev = &fpga->i2c->dev;
	struct vc_rom_table const	*tbl = &fpga->rom_table;
	size_t				i;

	dev_info(dev, "magic=%.*s, manuf=%.*s (%04x), sensor=%.*s (%.*s), module=%04x.%0x\n",
		 (int)(sizeof(tbl->magic)), tbl->magic,
		 (int)(sizeof(tbl->manuf)), tbl->manuf, tbl->manuf_id,
		 (int)(sizeof(tbl->sen_manuf)), tbl->sen_manuf,
		 (int)(sizeof(tbl->sen_type)), tbl->sen_type,
		 tbl->mod_id, tbl->mod_rev);

	for (i = 0; i < tbl->nr_modes; ++i) {
		dev_dbg(dev, "  mode#%zu: %*ph\n", i,
			min_t(int, sizeof(tbl->mode1), tbl->bytes_per_mode),
			i == 0 ? tbl->mode1 :
			i == 1 ? tbl->mode2 :
			i == 2 ? tbl->mode3 :
			i == 3 ? tbl->mode4 : NULL);
	}
}

static int vc_fpga_parse_rom_table(struct vc_fpga *fpga)
{
	struct vc_rom_table const	*tbl = &fpga->rom_table;
	struct vc_rom_info		*info = &fpga->rom_info;
	size_t				len;

	len = strnlen(tbl->sen_type, sizeof(tbl->sen_type));
	if (len > 0 && len <= sizeof(tbl->sen_type))
		info->is_color = (tbl->sen_type[len - 1] == 'C');
	else
		info->is_color = false;

	switch (tbl->mod_id) {
	case 0x9281:
		info->sensor = VC_ROM_SENSOR_OV9281;
		break;

	case 0x0327:
		info->sensor = VC_ROM_SENSOR_IMX327;
		break;

	default:
		dev_warn(&fpga->i2c->dev, "unsupported model %04x\n",
			 tbl->mod_id);
		return -EINVAL;
	}

	info->tbl = tbl;
	info->reset_on_pwroff = true;	/* TODO: configure in dtree? */

	return 0;
}

struct vc_rom_info const *vc_fpga_rom_info(struct vc_fpga const *fpga)
{
	return &fpga->rom_info;
}
EXPORT_SYMBOL_GPL(vc_fpga_rom_info);

struct vc_fpga *of_vc_fpga_find(struct device_node *node, char const *attr,
				int idx)
{
	struct device_node	*handle;
	struct i2c_client	*client;
	struct vc_fpga		*fpga;

	handle = of_parse_phandle(node, attr, idx);
	if (!handle) {
		pr_debug("%s: missing/bad phandle '%s' at %pOF\n",
			 __func__, attr, node);
		return ERR_PTR(-ENOENT);
	}

	client = of_find_i2c_device_by_node(handle);
	if (!client) {
		pr_debug("%s: missing device for  %pOF\n", __func__, handle);
		of_node_put(handle);
		return ERR_PTR(-ENOENT);
	}

	of_node_put(handle);

	fpga = i2c_get_clientdata(client);
	if (!fpga) {
		pr_debug("%s: fpga not initialized yet\n", __func__);
		put_device(&client->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	return fpga;
}
EXPORT_SYMBOL_GPL(of_vc_fpga_find);

void vc_fpga_put(struct vc_fpga *fpga)
{
	if (IS_ERR_OR_NULL(fpga))
		return;

	put_device(&fpga->i2c->dev);
}
EXPORT_SYMBOL_GPL(vc_fpga_put);

static int _vc_fpga_set_r100(struct vc_fpga *fpga, uint8_t clr, uint8_t set,
			     bool *changed_ptr)
{
	int			rc;
	uint8_t			val;
	bool			changed = false;

	val = fpga->shadow_r100;

	val &= ~clr;
	val |= set;

	if (val != fpga->shadow_r100) {
		rc = vc_fpga_write(fpga, 0x100, val);
		if (rc < 0)
			goto out;

		fpga->shadow_r100 = val;
		changed = true;
	}

	rc = 0;

out:
	if (changed_ptr)
		*changed_ptr = changed;

	return rc;
}

static int _vc_fpga_wait_init(struct vc_fpga *fpga)
{
	unsigned int	try;
	int		rc;
	bool		exit_wait = false;

	for (try = 100; try > 0; --try) {
		uint8_t		status;

		rc = vc_fpga_read(fpga, 0x101, &status);
		if (rc < 0)
			break;

		if (status & BIT(0)) {
			dev_warn(&fpga->i2c->dev, "initialization sequence error\n");
			rc = -EIO;
			break;
		}

		if (status & BIT(7)) {
			rc = 0;
			break;
		}

		exit_wait = true;
		usleep_range(10000, 15000);
		rc = -ETIMEDOUT;
	}

	if (rc < 0)
		goto out;

	if (exit_wait)
		usleep_range(10000, 15000);

out:
	return rc;
}

static int _vc_fpga_write_cfg(struct vc_fpga *fpga)
{
	int			rc = 0;

	if (fpga->need_cfg) {
		rc = vc_fpga_write(fpga, 0x102, fpga->cfg_num_code);
		if (rc < 0)
			goto out;

		fpga->need_cfg = false;
	}

out:
	return rc;
}

static int _vc_fpga_regulator_set(struct regulator_dev *rdev, bool enable)
{
	struct vc_fpga		*fpga = rdev_get_drvdata(rdev);
	int			rc;
	uint8_t			r100_clr;
	uint8_t			r100_set;

	mutex_lock(&fpga->mode_lock);
	if (enable) {
		rc = _vc_fpga_write_cfg(fpga);
		if (rc < 0)
			goto out;
	}

	r100_clr = !enable ? 0 : R100_PWROFF;
	r100_set = enable  ? 0 : R100_PWROFF;

	if (fpga->rom_info.reset_on_pwroff) {
		if (!fpga->is_reset && enable)
			r100_clr |= R100_RESET;
		else
			r100_set |= R100_RESET;
	}

	rc =  _vc_fpga_set_r100(fpga, r100_clr, r100_set, NULL);
	if (rc < 0)
		goto out;

	if (fpga->shadow_r100 == 0) {
		rc = _vc_fpga_wait_init(fpga);
		if (rc < 0)
			goto out;
	}

	rc = 0;

out:
	mutex_unlock(&fpga->mode_lock);

	return rc;
}

static int vc_fpga_regulator_enable(struct regulator_dev *rdev)
{
	return _vc_fpga_regulator_set(rdev, true);
}

static int vc_fpga_regulator_disable(struct regulator_dev *rdev)
{
	return _vc_fpga_regulator_set(rdev, false);
}

static int vc_fpga_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct vc_fpga		*fpga = rdev_get_drvdata(rdev);

	uint8_t	v;
	int	rc  = vc_fpga_read(fpga, 0x100, &v);

	if (rc < 0)
		return rc;

	return (v & BIT(1)) ? 0 : 1;
}

static int vc_fpga_regulator_get_voltage(struct regulator_dev *reg)
{
	/* TODO */
	return 380000000;
}

static struct regulator_ops const vc_fpga_regulator_ops = {
	.enable			= vc_fpga_regulator_enable,
	.disable		= vc_fpga_regulator_disable,
	.is_enabled		= vc_fpga_regulator_is_enabled,
	.get_voltage		= vc_fpga_regulator_get_voltage,
};

static struct regulator_desc const vc_fpga_regulator_desc = {
	.name			= "vc-fpga",
	.supply_name		= "vin",
	.ops			= &vc_fpga_regulator_ops,
	.owner			= THIS_MODULE,
	.type			= REGULATOR_VOLTAGE,
	.n_voltages		= 1,
	.off_on_delay		= 200000,
};

static int vc_fpga_init_regulator(struct vc_fpga *fpga)
{
	struct device			*dev = &fpga->i2c->dev;
	struct device_node		*np;
	struct regulator_dev		*rdev;
	int				rc;

	np = of_get_child_by_name(dev->of_node, "regulator");
	if (!np || !of_device_is_available(np)) {
		dev_info(dev, "missing regulator node; will not provide regulator\n");
		return 0;
	}

	{
		struct regulator_config		config = {
			.dev		= dev,
			.driver_data	= fpga,
			.of_node	= np,
		};

		rdev = devm_regulator_register(dev, &vc_fpga_regulator_desc,
					       &config);

		rc = PTR_ERR_OR_ZERO(rdev);
	}

	rdev->constraints->valid_ops_mask |= REGULATOR_CHANGE_STATUS;

	of_node_put(np);

	if (rc < 0)
		goto out;

	fpga->rdev = rdev;

	rc = 0;

out:
	return rc;
}

static int _vc_fpga_reset_assert(struct vc_fpga *fpga)
{
	int	rc;

	rc = _vc_fpga_set_r100(fpga, 0, R100_RESET, NULL);
	if (rc < 0)
		goto out;

	fpga->is_reset = true;
	rc = 0;

out:
	return rc;
}

static int _vc_fpga_reset_deassert(struct vc_fpga *fpga)
{
	int		rc;

	rc = _vc_fpga_set_r100(fpga, R100_RESET, 0, NULL);
	if (rc < 0)
		goto out;

	if (fpga->shadow_r100 == 0) {
		rc = _vc_fpga_wait_init(fpga);
		if (rc < 0)
			goto out;
	}

	fpga->is_reset = false;
	rc = 0;

out:
	return rc;
}

static int vc_fpga_reset_reset(struct reset_controller_dev *rdev,
			       unsigned long _id)
{
	struct vc_fpga		*fpga = reset_to_fpga(rdev);
	int			rc;

	mutex_lock(&fpga->mode_lock);
	rc = _vc_fpga_reset_assert(fpga);
	if (rc < 0)
		goto out;

	rc = _vc_fpga_write_cfg(fpga);
	if (rc < 0)
		goto out;

	usleep_range(1000, 2000);

	rc = _vc_fpga_reset_deassert(fpga);
	if (rc < 0)
		goto out;

	rc = 0;

out:
	mutex_unlock(&fpga->mode_lock);

	return rc;
}

static int vc_fpga_reset_status(struct reset_controller_dev *rdev,
				unsigned long _id)
{
	struct vc_fpga		*fpga = reset_to_fpga(rdev);

	uint8_t	v;
	int	rc  = vc_fpga_read(fpga, 0x100, &v);

	if (rc < 0)
		return rc;

	return (v & R100_RESET) ? 1 : 0;
}

static int of_vc_fpga_reset_xlate(struct reset_controller_dev *rcdev,
				  const struct of_phandle_args *reset_spec)
{
	return 0;
}

static struct reset_control_ops const vc_fpga_reset_ops = {
	.reset			= vc_fpga_reset_reset,
	.status			= vc_fpga_reset_status,
};

static int vc_fpga_init_reset_controller(struct vc_fpga *fpga)
{
	struct device			*dev = &fpga->i2c->dev;
	struct device_node		*np;
	int				rc;

	np = of_get_child_by_name(dev->of_node, "reset");
	if (!np || !of_device_is_available(np)) {
		dev_info(dev, "missing 'reset' node; will not provide reset controller\n");
		return 0;
	}

	fpga->reset_ctrl = (struct reset_controller_dev) {
		.owner		= THIS_MODULE,
		.ops		= &vc_fpga_reset_ops,
		.nr_resets	= 1,
		.of_node	= np,
		.of_xlate	= of_vc_fpga_reset_xlate,
	};

	rc = devm_reset_controller_register(dev, &fpga->reset_ctrl);
	if (rc < 0) {
		dev_err(dev, "failed to register reset controller: %d\n", rc);
		goto out;
	}

	rc = 0;

out:
	of_node_put(np);

	return rc;
}

static int vc_fpga_gpioc_get_direction(struct gpio_chip *chip,
				       unsigned int ofs)
{
	if (WARN_ON(ofs) > 0)
		return -EINVAL;

	return 1;
}

static void vc_fpga_gpioc_set(struct gpio_chip *chip,
			     unsigned int ofs, int val)
{
	struct vc_fpga			*fpga = gpioc_to_fpga(chip);
	int				rc;

	if (WARN_ON(ofs) > 0)
		return;

	mutex_lock(&fpga->mode_lock);
	if (val && !(fpga->shadow_r100 & R100_RESET)) {
		rc = _vc_fpga_reset_assert(fpga);
	} else if (!val && (fpga->shadow_r100 & R100_RESET)) {
		rc = _vc_fpga_write_cfg(fpga);
		if (rc >= 0) {
			usleep_range(1000, 2000);
			rc = _vc_fpga_reset_deassert(fpga);
		}
	} else {
		rc = 0;
		/* noop */
	}
	mutex_unlock(&fpga->mode_lock);

	if (rc < 0)
		dev_warn(&fpga->i2c->dev, "failed to %s reset gpio: %d\n",
			 val ? "assert" : "deassert", rc);
}

static int vc_fpga_gpioc_get(struct gpio_chip *chip, unsigned int ofs)
{
	struct vc_fpga			*fpga = gpioc_to_fpga(chip);
	int				rc;

	if (WARN_ON(ofs) > 0)
		return 0;

	mutex_lock(&fpga->mode_lock);
	rc = (fpga->shadow_r100 & R100_RESET) != 0;
	mutex_unlock(&fpga->mode_lock);

	return rc;
}

static int vc_fpga_gpioc_direction_output(struct gpio_chip *chip,
					  unsigned int ofs, int val)
{
	if (WARN_ON(ofs) > 0)
		return -EINVAL;

	vc_fpga_gpioc_set(chip, ofs, val);

	return 0;
}

static int vc_fpga_init_gpio_chip(struct vc_fpga *fpga)
{
	struct device			*dev = &fpga->i2c->dev;
	struct device_node		*np;
	int				rc;

	np = of_get_child_by_name(dev->of_node, "gpio-chip");
	if (!np || !of_device_is_available(np)) {
		dev_info(dev, "missing 'reset' node; will not provide gpio chip\n");
		return 0;
	}

	fpga->gpioc = (struct gpio_chip) {
		.label			= "vc-fpga",
		.parent			= dev,
		.of_node		= np,
		.owner			= THIS_MODULE,
		.names			= (char const *[]) {
			[0]		= "reset",
		},
		.base			= -1,
		.ngpio			= 1,
		.can_sleep		= true,
		.get_direction		= vc_fpga_gpioc_get_direction,
		.direction_output	= vc_fpga_gpioc_direction_output,
		.set			= vc_fpga_gpioc_set,
		.get			= vc_fpga_gpioc_get,
	};

	rc = devm_gpiochip_add_data(dev, &fpga->gpioc, fpga);

	of_node_put(np);

	return rc;
}

static int vc_fpga_i2c_probe(struct i2c_client *i2c,
			     struct i2c_device_id const *devid)
{
	struct vc_fpga	*fpga;
	int		rc;
	size_t		i;

	fpga = devm_kzalloc(&i2c->dev, sizeof(*fpga), GFP_KERNEL);
	if (!fpga)
		return -ENOMEM;

	fpga->i2c = i2c;
	fpga->need_cfg = true;
	fpga->is_reset = true;

	mutex_init(&fpga->mode_lock);

	for (i = 0; i < sizeof(fpga->rom_table); ++i) {
		void	*ptr = &fpga->rom_table;

		rc = vc_fpga_read(fpga, 0x1000 + i, ptr + i);
		if (rc < 0)
			goto out;
	}

	rc = vc_fpga_parse_rom_table(fpga);
	if (rc < 0)
		goto out;

	/* power off device on start */
	rc =  _vc_fpga_set_r100(fpga, 0, R100_RESET | R100_PWROFF, NULL);
	if (rc < 0)
		goto out;

	rc = vc_fpga_init_regulator(fpga);
	if (rc < 0)
		goto out;

	rc = vc_fpga_init_reset_controller(fpga);
	if (rc < 0)
		goto out;

	rc = vc_fpga_init_gpio_chip(fpga);
	if (rc < 0)
		goto out;

	vc_fpga_show_info(fpga);

	i2c_set_clientdata(i2c, fpga);

	rc = 0;

out:
	return rc;
}

static int vc_fpga_i2c_remove(struct i2c_client *i2c)
{
	i2c_set_clientdata(i2c, NULL);
	return 0;
}

static struct of_device_id const vc_fpga_of_match[] = {
	{.compatible = "vc,fpga" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vc_fpga_of_match);

static struct i2c_device_id const vc_fpga_ids[] = {
	{"vc_fpga", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, vc_fpga_ids);

static struct i2c_driver vc_fpga_i2c_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "vc_fpga",
		.of_match_table	= of_match_ptr(vc_fpga_of_match),
	},

	.probe =	vc_fpga_i2c_probe,
	.remove =	vc_fpga_i2c_remove,
	.id_table =	vc_fpga_ids,
};
module_i2c_driver(vc_fpga_i2c_driver);

MODULE_AUTHOR("Enrico Scholz <enrico.scholz@sigma-chemnitz.de>");
MODULE_DESCRIPTION("VC FPGA Camera Driver");
MODULE_LICENSE("GPL");
