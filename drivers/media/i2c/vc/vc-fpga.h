/* SPDX-License-Identifier: GPL-2.0 */

#ifndef H_LINUX_DRIVERS_MEDIA_I2C_VC_VC_FPGA_H
#define H_LINUX_DRIVERS_MEDIA_I2C_VC_VC_FPGA_H

#include <linux/types.h>

struct vc_rom_table {
	char magic[12];
	char manuf[32];
	u16 manuf_id;
	char sen_manuf[8];
	char sen_type[16];
	u16 mod_id;
	u16 mod_rev;
	char regs[56];
	u16 nr_modes;
	u16 bytes_per_mode;
	char mode1[16];
	char mode2[16];
	char mode3[16];
	char mode4[16];
};

enum vc_rom_sensor {
	VC_ROM_SENSOR_OV9281,
	VC_ROM_SENSOR_IMX327,
};

struct vc_rom_info {
	struct vc_rom_table const	*tbl;
	bool				is_color;
	enum vc_rom_sensor		sensor;
	bool				reset_on_pwroff;
};

enum vc_fpga_cfg_action {
	VC_FPGA_CFG_SAME,
	VC_FPGA_CFG_NEED_RESET,
};

struct vc_fpga;
struct device_node;

struct vc_fpga *of_vc_fpga_find(struct device_node *np, char const *attr,
				int idx);

void vc_fpga_put(struct vc_fpga *fpga);

struct vc_rom_info const *vc_fpga_rom_info(struct vc_fpga const *fpga);

enum vc_fpga_cfg_action vc_fpga_cfg(struct vc_fpga *fpga, uint8_t mode);

#endif	/* H_LINUX_DRIVERS_MEDIA_I2C_VC_VC_FPGA_H */
