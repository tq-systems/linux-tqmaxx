/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

#include <linux/linkage.h>
#include "hardware.h"

#ifdef CONFIG_SMP

	.align 3

	.macro	disable_l1_dcache

	/*
	 * Flush all data from the L1 data cache before disabling
	 * SCTLR.C bit.
	 */
	push	{r0 - r11, lr}

	ldr	r7, =v7_flush_kern_cache_all
	mov	lr, pc
	mov	pc, r7
	pop	{r0 - r11, lr}

	/* disable d-cache */
	mrc	p15, 0, r6, c1, c0, 0
	bic	r6, r6, #0x4
	mcr	p15, 0, r6, c1, c0, 0
	dsb
	isb

	push	{r0 - r11, lr}

	ldr	r7, =v7_flush_kern_cache_all
	mov	lr, pc
	mov	pc, r7
	pop	{r0 - r11, lr}

	.endm

/**
 * @brief  Switch CPU in WFE mode while bus frequency change
 *         on-going
 *
 * @param[in] r0  CPU in WFE Status
 * @param[in] r1  Bus frequency change status
 */

.globl imx_smp_wfe_optee_end

ENTRY(imx_smp_wfe_optee)
	push  {r4-r11, lr}

	dsb
	isb

	disable_l1_dcache
	isb

	/* Set flag CPU entering WFE. */
	mov   r4, #1
	str   r4, [r0]

	dsb
	isb

1:
	wfe

	/* Check if busfreq is done, else loop */
	ldr   r4, [r1]
	cmp   r4, #1
	beq   1b

	/* Enable L1 data cache. */
	mrc   p15, 0, r4, c1, c0, 0
	orr   r4, r4, #0x4
	mcr   p15, 0, r4, c1, c0, 0
	isb

	/* Set flag CPU exiting WFE. */
	mov   r4, #0
	str   r4, [r0]

	/* Pop all saved registers. */
	pop   {r4-r11, lr}
	mov   pc, lr
	.ltorg
imx_smp_wfe_optee_end:
ENDPROC(imx_smp_wfe_optee)
#endif
