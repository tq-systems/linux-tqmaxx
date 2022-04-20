// SPDX-License-Identifier: GPL-2.0
/*
 * TI AM64 temperature sensor driver
 *
 * Copyright (C) 2022 TQ-Systems GmbH
 * Author: Matthias Schiffer
 *
 * Based on:
 *
 * TI Bandgap temperature sensor driver for K3 SoC Family
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/thermal.h>
#include <linux/types.h>

#include "thermal_hwmon.h"

#define AM64_VTM_CFG1_DEVINFO_PWR0_OFFSET		0x4
#define AM64_VTM_CFG1_DEVINFO_PWR0_TEMPSENS_CT_MASK	0x000000f0

#define AM64_VTM_CFG1_TMPSENS_STAT_0_OFFSET		0x308
#define AM64_VTM_CFG1_REGS_PER_TS			0x20
#define AM64_VTM_CFG1_TMPSENS_STAT_DATA_OUT_MASK	0x000003ff

#define AM64_VTM_CFG2_TMPSENS_CTRL_0_OFFSET		0x300
#define AM64_VTM_CFG2_REGS_PER_TS			0x20
#define AM64_VTM_CFG2_TMPSENS_CTRL_CONT			BIT(4)
#define AM64_VTM_CFG2_TMPSENS_CTRL_SOC			BIT(5)
#define AM64_VTM_CFG2_TMPSENS_CTRL_CLRZ			BIT(6)

struct am64_thermal_sensor {
	void __iomem *reg_ctrl;
	void __iomem *reg_stat;
};

static int am64_thermal_convert_temp(u32 val)
{
	/* Temperature in °C is computed as:
	 *
	 *   a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0
	 *
	 * where
	 *
	 *   a4 = -9.2627E-12
	 *   a3 = 6.0373E-08
	 *   a2 = -1.7058E-04
	 *   a1 = 3.2512E-01
	 *   a0 = -4.9002E+01
	 *
	 * Coefficient values are scaled by 10^16 in the following code.
	 * Scaling is chosen so that no precision is lost before the final
	 * division and no intermediate value can overflow the s64 temp for
	 * a 10bit input value.
	 */
	const s64 a4 = -92627ll,
		  a3 = 603730000ll,
		  a2 = -1705800000000ll,
		  a1 = 3251200000000000ll,
		  a0 = -490020000000000000ll;
	s64 temp, x;

	temp = a0;
	x = val; temp += a1 * x;
	x *= val; temp += a2 * x;
	x *= val; temp += a3 * x;
	x *= val; temp += a4 * x;

	/* Divide by 10^13 to get result in °C/1000 */
	return div64_s64(temp, 10000000000000ll);
}

static int am64_thermal_get_temp(void *data, int *temp)
{
	struct am64_thermal_sensor *sensor = data;
	u32 val;

	val = readl(sensor->reg_stat);
	val &= AM64_VTM_CFG1_TMPSENS_STAT_DATA_OUT_MASK;

	*temp = am64_thermal_convert_temp(val);

	return 0;
}

static const struct thermal_zone_of_device_ops am64_of_thermal_ops = {
	.get_temp = am64_thermal_get_temp,
};

static int am64_thermal_add_sensor(struct device *dev,
				   void __iomem *cfg1, void __iomem *cfg2,
				   struct am64_thermal_sensor *sensor, int id)
{
	struct thermal_zone_device *tzd;
	u32 val;

	sensor->reg_ctrl = cfg2 + AM64_VTM_CFG2_TMPSENS_CTRL_0_OFFSET +
			   id * AM64_VTM_CFG2_REGS_PER_TS;
	sensor->reg_stat = cfg1 + AM64_VTM_CFG1_TMPSENS_STAT_0_OFFSET +
			   id * AM64_VTM_CFG1_REGS_PER_TS;

	val = readl(sensor->reg_ctrl);
	val |= AM64_VTM_CFG2_TMPSENS_CTRL_CONT |
	       AM64_VTM_CFG2_TMPSENS_CTRL_CLRZ;
	writel(val, sensor->reg_ctrl);

	tzd = devm_thermal_zone_of_sensor_register(dev, id, sensor,
						   &am64_of_thermal_ops);
	if (IS_ERR(tzd)) {
		dev_err(dev, "Failed to register sensor %d: %pe\n", id, tzd);
		return PTR_ERR(tzd);
	}

	if (devm_thermal_add_hwmon_sysfs(tzd))
		dev_warn(dev, "Failed to add hwmon sysfs attributes\n");

	return 0;
}

static int am64_thermal_probe(struct platform_device *pdev)
{
	struct am64_thermal_sensor *sensors;
	struct device *dev = &pdev->dev;
	struct resource *res1, *res2;
	void __iomem *cfg1, *cfg2;
	int ret, cnt, id;

	res1 = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg1");
	cfg1 = devm_ioremap_resource(dev, res1);
	if (IS_ERR(cfg1))
		return PTR_ERR(cfg1);

	res2 = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg2");
	cfg2 = devm_ioremap_resource(dev, res2);
	if (IS_ERR(cfg2))
		return PTR_ERR(cfg2);

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		pm_runtime_disable(dev);
		return ret;
	}

	/* Get the sensor count in the VTM */
	cnt = readl(cfg1 + AM64_VTM_CFG1_DEVINFO_PWR0_OFFSET);
	cnt &= AM64_VTM_CFG1_DEVINFO_PWR0_TEMPSENS_CT_MASK;
	cnt >>= __ffs(AM64_VTM_CFG1_DEVINFO_PWR0_TEMPSENS_CT_MASK);

	sensors = devm_kcalloc(dev, cnt, sizeof(*sensors), GFP_KERNEL);
	if (!sensors) {
		ret = -ENOMEM;
		goto err;
	}

	/* Register the thermal sensors */
	for (id = 0; id < cnt; id++) {
		ret = am64_thermal_add_sensor(dev, cfg1, cfg2, &sensors[id],
					      id);
		if (ret < 0)
			goto err;
	}

	return 0;

err:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	return ret;
}

static int am64_thermal_remove(struct platform_device *pdev)
{
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct of_device_id of_am64_thermal_match[] = {
	{
		.compatible = "ti,am64-vtm",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, of_am64_thermal_match);

static struct platform_driver am64_thermal_sensor_driver = {
	.probe = am64_thermal_probe,
	.remove = am64_thermal_remove,
	.driver = {
		.name = "am64-thermal",
		.of_match_table	= of_am64_thermal_match,
	},
};

module_platform_driver(am64_thermal_sensor_driver);

MODULE_DESCRIPTION("AM64 temperature sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Matthias Schiffer <matthias.schiffer@ew.tq-group.com>");
