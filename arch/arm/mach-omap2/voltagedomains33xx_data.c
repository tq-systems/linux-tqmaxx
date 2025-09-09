// SPDX-License-Identifier: GPL-2.0-only
/*
 * AM33XX voltage domain data
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/kernel.h>
#include <linux/init.h>

#include "voltage.h"

static struct voltagedomain am33xx_voltdm_mpu = {
	.name		= "mpu",
};

static struct voltagedomain am33xx_voltdm_core = {
	.name		= "core",
};

static struct voltagedomain am33xx_voltdm_rtc = {
	.name		= "rtc",
};

static struct voltagedomain *voltagedomains_am33xx[] __initdata = {
	&am33xx_voltdm_mpu,
	&am33xx_voltdm_core,
	&am33xx_voltdm_rtc,
	NULL,
};

void __init am33xx_voltagedomains_init(void)
{
	voltdm_init(voltagedomains_am33xx);
}
