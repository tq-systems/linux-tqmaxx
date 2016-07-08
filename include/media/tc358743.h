/*
 * tc358743 - Toshiba HDMI to CSI-2 bridge
 *
 * Copyright 2014 Cisco Systems, Inc. and/or its affiliates. All rights
 * reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/*
 * References (c = chapter, p = page):
 * REF_01 - Toshiba, TC358743XBG (H2C), Functional Specification, Rev 0.60
 * REF_02 - Toshiba, TC358743XBG_HDMI-CSI_Tv11p_nm.xls
 */

#ifndef _TC358743_
#define _TC358743_

enum tc358743_ddc5v_delays {
	DDC5V_DELAY_0MS,
	DDC5V_DELAY_50MS,
	DDC5V_DELAY_100MS,
	DDC5V_DELAY_200MS,
};

struct tc358743_platform_data {
	/* System clock connected to REFCLK (pin H5) */
	u32 refclk_hz; /* 26 MHz, 27 MHz or 42 MHz */

	/* DDC +5V debounce delay */
	enum tc358743_ddc5v_delays ddc5v_delay;

	bool enable_hdcp;

	/*
	 * The FIFO size is 512x32, so Toshiba recommend to set the default FIFO
	 * level to somewhere in the middle (e.g. 300), so it can cover speed
	 * mismatches in input and output ports.
	 */
	u16 fifo_level;

	/* Bps pr lane is (refclk_hz / pll_prd) * pll_fbd */
	u16 pll_prd;
	u16 pll_fbd;

	/* CSI
	 * Calculate CSI parameters with REF_02 for the highest resolution your
	 * CSI interface can handle. The driver will adjust the number of CSI
	 * lanes in use according to the pixel clock.
	 *
	 * The values in brackets are calculated with REF_02 when the number of
	 * bps pr lane is 823.5 MHz, and can serve as a starting point.
	 */
	u32 lineinitcnt;	/* (0x00001770) */
	u32 lptxtimecnt;	/* (0x00000005) */
	u32 tclk_headercnt;	/* (0x00001d04) */
	u32 tclk_trailcnt;	/* (0x00000000) */
	u32 ths_headercnt;	/* (0x00000505) */
	u32 twakeup;		/* (0x00004650) */
	u32 tclk_postcnt;	/* (0x00000000) */
	u32 ths_trailcnt;	/* (0x00000004) */
	u32 hstxvregcnt;	/* (0x00000005) */

	/* HDMI PHY */
	u8 phy_auto_rst; /* PHY_CTL2 */
	u8 hdmi_det_v; /* HDMI_DET */
	bool h_pi_rst; /* HV_RST */
	bool v_pi_rst; /* HV_RST */
};

/* notify events */
#define TC358743_FMT_CHANGE     1

/* custom controls */
/* Audio sample rate in Hz */
#define TC358743_CID_AUDIO_SAMPLING_RATE (V4L2_CID_USER_TC358743_BASE + 0)
/* Audio present status */
#define TC358743_CID_AUDIO_PRESENT       (V4L2_CID_USER_TC358743_BASE + 1)

#endif
