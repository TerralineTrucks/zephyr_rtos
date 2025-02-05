/*
 * Copyright (c) 2023 Arm Limited (or its affiliates). All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MACRO_PRIV_INC_
#define _MACRO_PRIV_INC_

#include <zephyr/arch/arm/cortex_a_r/tpidruro.h>

/*
 * Get CPU id
 */

.macro get_cpu_id rreg0
	/* Read MPIDR register */
	mrc p15, 0, \rreg0, c0, c0, 5
	ubfx	\rreg0, \rreg0, #0, #24
.endm

.macro get_cpu rreg0
	/*
         * Get CPU pointer.
	 */
	mrc p15, 0, \rreg0, c13, c0, 3
        and \rreg0, #TPIDRURO_CURR_CPU
.endm

.macro fpu_exc_exit
#if defined(CONFIG_FPU_SHARING)
	/*
	 * If the floating point context pointer is null, then a context was
	 * saved so restore the float context from the exception stack frame.
	 */
	get_cpu r2
	ldr r1, [r2, #___cpu_t_fp_ctx_OFFSET]
	cmp r1, #0
	beq vfp_restore\@

	/*
	 * If leaving the last interrupt context, remove the floating point
	 * context pointer.
	 */
	cmp r0, #0
	moveq r1, #0
	streq r1, [r2, #___cpu_t_fp_ctx_OFFSET]
	b vfp_exit\@

vfp_restore\@:
	add r3, sp, #___fpu_sf_t_fpscr_OFFSET
	ldm r3, {r1, r2}
	tst r2, #FPEXC_EN
	beq vfp_exit\@

	vmsr fpexc, r2
	vmsr fpscr, r1
	mov r3, sp
	vldmia r3!, {s0-s15}
#ifdef CONFIG_VFP_FEATURE_REGS_S64_D32
	vldmia r3!, {d16-d31}
#endif

vfp_exit\@:
	/* Leave the VFP disabled when leaving */
	mov r1, #0
	vmsr fpexc, r1

	add sp, sp, #___fpu_t_SIZEOF
#endif
.endm

.macro z_arm_cortex_ar_enter_exc
	/*
	 * Store r0-r3, r12, lr into the stack to construct an exception
	 * stack frame.
	 */
	srsdb sp!, #MODE_SYS
	cps #MODE_SYS
	stmdb sp, {r0-r3, r12, lr}^
	sub sp, #24

	/* TODO: EXTRA_EXCEPTION_INFO */
	mov r0, sp

	/* increment exception depth */
	get_cpu r2
	ldrb r1, [r2, #_cpu_offset_to_exc_depth]
	add r1, r1, #1
	strb r1, [r2, #_cpu_offset_to_exc_depth]
.endm

#endif /* _MACRO_PRIV_INC_ */
