/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * DSP Audio DAI header
 *
 * Copyright 2018 NXP
 */

#ifndef __FSL_DSP_CPU_H
#define __FSL_DSP_CPU_H

struct fsl_dsp_audio {
	struct platform_device *pdev;
};

extern struct platform_driver dsp_audio_driver;

#endif /*__FSL_DSP_CPU_H*/

