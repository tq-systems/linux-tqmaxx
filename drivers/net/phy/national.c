/*
 * drivers/net/phy/national.c
 *
 * Driver for National Semiconductor PHYs
 *
 * Author: Stuart Menefy <stuart.menefy@st.com>
 * Maintainer: Giuseppe Cavallaro <peppe.cavallaro@st.com>
 *
 * Copyright (c) 2008 STMicroelectronics Limited
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/of.h>

#define DEBUG

/* DP83865 phy identifier values */
#define DP83865_PHY_ID	0x20005c7a

#define DP83865_INT_STATUS	0x14
#define DP83865_INT_MASK	0x15
#define DP83865_INT_CLEAR	0x17

#define DP83865_INT_REMOTE_FAULT 0x0008
#define DP83865_INT_ANE_COMPLETED 0x0010
#define DP83865_INT_LINK_CHANGE	0xe000
#define DP83865_INT_MASK_DEFAULT (DP83865_INT_REMOTE_FAULT | \
				DP83865_INT_ANE_COMPLETED | \
				DP83865_INT_LINK_CHANGE)

/* Advanced proprietary configuration */
#define NS_EXP_MEM_CTL	0x16
#define NS_EXP_MEM_DATA	0x1d
#define NS_EXP_MEM_ADD	0x1e

#define LED_CTRL_REG 0x13
#define AN_FALLBACK_AN 0x0001
#define AN_FALLBACK_CRC 0x0002
#define AN_FALLBACK_IE 0x0004
#define ALL_FALLBACK_ON (AN_FALLBACK_AN |  AN_FALLBACK_CRC | AN_FALLBACK_IE)

enum hdx_loopback {
	hdx_loopback_on = 0,
	hdx_loopback_off = 1,
};

static u8 ns_exp_read(struct phy_device *phydev, u16 reg)
{
	phy_write(phydev, NS_EXP_MEM_ADD, reg);
	return phy_read(phydev, NS_EXP_MEM_DATA);
}

static void ns_exp_write(struct phy_device *phydev, u16 reg, u8 data)
{
	phy_write(phydev, NS_EXP_MEM_ADD, reg);
	phy_write(phydev, NS_EXP_MEM_DATA, data);
}

static int ns_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, DP83865_INT_MASK,
				DP83865_INT_MASK_DEFAULT);
	else
		err = phy_write(phydev, DP83865_INT_MASK, 0);

	return err;
}

static int ns_ack_interrupt(struct phy_device *phydev)
{
	int ret = phy_read(phydev, DP83865_INT_STATUS);
	if (ret < 0)
		return ret;

	/* Clear the interrupt status bit by writing a “1”
	 * to the corresponding bit in INT_CLEAR (2:0 are reserved) */
	ret = phy_write(phydev, DP83865_INT_CLEAR, ret & ~0x7);

	return ret;
}

static void ns_giga_speed_fallback(struct phy_device *phydev, int mode)
{
	int bmcr = phy_read(phydev, MII_BMCR);

	phy_write(phydev, MII_BMCR, (bmcr | BMCR_PDOWN));

	/* Enable 8 bit expended memory read/write (no auto increment) */
	phy_write(phydev, NS_EXP_MEM_CTL, 0);
	phy_write(phydev, NS_EXP_MEM_ADD, 0x1C0);
	phy_write(phydev, NS_EXP_MEM_DATA, 0x0008);
	phy_write(phydev, MII_BMCR, (bmcr & ~BMCR_PDOWN));
	phy_write(phydev, LED_CTRL_REG, mode);
}

static void ns_10_base_t_hdx_loopack(struct phy_device *phydev, int disable)
{
	if (disable)
		ns_exp_write(phydev, 0x1c0, ns_exp_read(phydev, 0x1c0) | 1);
	else
		ns_exp_write(phydev, 0x1c0,
			     ns_exp_read(phydev, 0x1c0) & 0xfffe);

	pr_debug("10BASE-T HDX loopback %s\n",
		 (ns_exp_read(phydev, 0x1c0) & 0x0001) ? "off" : "on");
}

static int ns_config_init(struct phy_device *phydev)
{
	ns_giga_speed_fallback(phydev, ALL_FALLBACK_ON);
	/* In the latest MAC or switches design, the 10 Mbps loopback
	   is desired to be turned off. */
	ns_10_base_t_hdx_loopack(phydev, hdx_loopback_off);
	return ns_ack_interrupt(phydev);
}

/* DP83867 phy identifier values */
#define DP83867_PHY_ID	0x2000a230

#define DP83867_INT_CTRL	0x12
#define DP83867_INT_STATUS	0x13 /* Clear on read */

static int dp83867_ack_interrupt(struct phy_device *phydev)
{
	return phy_read(phydev, DP83867_INT_STATUS);
}

/* PHY Registers */
#define DP83867_MMD_REGCR			0x0d
#define DP83867_MMD_REGCR_DEVAD			0x001f

/* Register operations */
#define DP83867_MMD_REGCR_SET_ADDR		0x0000
/* Data operations */
#define DP83867_MMD_REGCR_DATA_NO_POST_INC	0x4000
#define DP83867_MMD_REGCR_DATA_POST_INC_RW	0x8000
#define DP83867_MMD_REGCR_DATA_POST_INC_W	0xC000

#define DP83867_MMD_ADDAR			0x0e

#define DP83867_LEDCR1				0x18
#define DP83867_LEDCR2				0x19

#define DP83867_RGMMIICTL			0x32
#define DP83867_RGMMIICTL_RX_DELAY		0x01
#define DP83867_RGMMIICTL_TX_DELAY		0x01
#define DP83867_RGMMIIDCTL			0x86
#define DP83867_RGMMIIDCTL_TX_DELAY		0xf0
#define DP83867_RGMMIIDCTL_RX_DELAY		0x0f

/* Accessors to extended registers*/
int dp83867_phy_extended_write(struct phy_device *phydev, int regnum,
			       u16 val)
{
	/*select register addr for mmd*/
	phy_write(phydev, DP83867_MMD_REGCR, DP83867_MMD_REGCR_DEVAD);
	/*select register for mmd*/
	phy_write(phydev, DP83867_MMD_ADDAR, regnum);
	/*setup mode*/
	phy_write(phydev, DP83867_MMD_REGCR,
		  (DP83867_MMD_REGCR_DATA_POST_INC_RW |
		   DP83867_MMD_REGCR_DEVAD));
	/*write the value*/
	return	phy_write(phydev, DP83867_MMD_ADDAR, val);
}

int dp83867_phy_extended_read(struct phy_device *phydev, int regnum)
{
	phy_write(phydev, DP83867_MMD_REGCR, DP83867_MMD_REGCR_DEVAD);
	phy_write(phydev, DP83867_MMD_ADDAR, regnum);
	phy_write(phydev, DP83867_MMD_REGCR,
		  (DP83867_MMD_REGCR_DEVAD |
		   DP83867_MMD_REGCR_DATA_NO_POST_INC));
	return phy_read(phydev, DP83867_MMD_ADDAR);
}

static inline int dp83867_ps_to_reg(int ps)
{
	return (ps > 4000) ? 0x0f : ((ps < 250) ? 0 : (ps / 250) - 1);
}

static void dp83867_load_values_from_of(struct phy_device *phydev,
				       struct device_node *of_node)
{
	int rx = -1;
	int tx = -1;
	int dm = 0;
	int rv = 0;

	if (!of_property_read_u32(of_node, "rx-skew-ps", &rx))
		dm |= DP83867_RGMMIICTL_RX_DELAY;

	if (!of_property_read_u32(of_node, "tx-skew-ps", &tx))
		dm |= DP83867_RGMMIICTL_TX_DELAY;

	if (dm & DP83867_RGMMIICTL_RX_DELAY)
		rv |= dp83867_ps_to_reg(rx);
	if (dm & DP83867_RGMMIICTL_TX_DELAY)
		rv |= (dp83867_ps_to_reg(tx) << 4);

	dp83867_phy_extended_write(phydev, DP83867_RGMMIIDCTL, rv);
	rv = dp83867_phy_extended_read(phydev, DP83867_RGMMIICTL);
	if (rv != -1) {
		rv &= ~(DP83867_RGMMIICTL_TX_DELAY |
			DP83867_RGMMIICTL_RX_DELAY);
		rv |= dm;
		dp83867_phy_extended_write(phydev, DP83867_RGMMIICTL, rv);
	}

	if (!of_property_read_u32(of_node, "led-cfg1", &rv))
		phy_write(phydev, DP83867_LEDCR1, rv);
	if (!of_property_read_u32(of_node, "led-cfg2", &rv))
		phy_write(phydev, DP83867_LEDCR2, rv);
}

static int dp83867_config_init(struct phy_device *phydev)
{
	struct device *dev = &phydev->dev;
	struct device_node *of_node = dev->of_node;

	if (!of_node && dev->parent->of_node)
		of_node = dev->parent->of_node;


	dp83867_ack_interrupt(phydev);
	dp83867_load_values_from_of(phydev, of_node);

	return 0;
}

static struct phy_driver national_phy_driver[] = {
{
	.phy_id = DP83865_PHY_ID,
	.phy_id_mask = 0xfffffff0,
	.name = "NatSemi DP83865",
	.features = PHY_GBIT_FEATURES | SUPPORTED_Pause | SUPPORTED_Asym_Pause,
	.flags = PHY_HAS_INTERRUPT,
	.config_init = ns_config_init,
	.config_aneg = genphy_config_aneg,
	.read_status = genphy_read_status,
	.ack_interrupt = ns_ack_interrupt,
	.config_intr = ns_config_intr,
	.driver = {.owner = THIS_MODULE,}
}, {
	.phy_id = DP83867_PHY_ID,
	.phy_id_mask = 0xfffffff0,
	.name = "NatSemi DP83867",
	.features = PHY_GBIT_FEATURES | SUPPORTED_Pause | SUPPORTED_Asym_Pause,
	.config_init = dp83867_config_init,
	.config_aneg = genphy_config_aneg,
	.read_status = genphy_read_status,
	.driver = {.owner = THIS_MODULE,}
} };

module_phy_driver(national_phy_driver);

MODULE_DESCRIPTION("NatSemi PHY driver");
MODULE_AUTHOR("Stuart Menefy");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused ns_tbl[] = {
	{ DP83865_PHY_ID, 0xfffffff0 },
	{ DP83867_PHY_ID, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, ns_tbl);
