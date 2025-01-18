/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ARM Cortex-R System Control Block interface
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <cmsis_core.h>
#include <zephyr/linker/linker-defs.h>

#if defined(CONFIG_ARM_MPU)
#if defined(CONFIG_CPU_HAS_ARM_MPU)
#if defined(CONFIG_CPU_CORTEX_R52)
/**
 *
 * @brief Clear all MPU region configuration
 *
 * This routine clears all ARM MPU region configuration.
 *
 */
void z_arm_clear_arm_mpu_config(void)
{
	int i;
	int num_regions = read_mpuir() & MPU_IR_REGION_Msk;

	/* First disable the MPU. Having an enabled MPU with no regions configured will
	 * trap the R52 core. */

	/* Force any outstanding transfers to complete before disabling MPU */
	barrier_dsync_fence_full();
	uint32_t val = __get_SCTLR();
	val &= ~SCTLR_MPU_ENABLE;
	__set_SCTLR(val);
	/* Make sure that all the registers are set before proceeding */
	barrier_dsync_fence_full();
	barrier_isync_fence_full();

	for (i = 0; i < num_regions; i++) {
		write_prselr(i);
		write_prbar(0);
		write_prlar(0);
	}
	barrier_dsync_fence_full();
}
#endif /* CONFIG_CPU_CORTEX_R52 */
#endif /* CONFIG_CPU_HAS_NXP_MPU */
#endif /* CONFIG_ARM_MPU */
