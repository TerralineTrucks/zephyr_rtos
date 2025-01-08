/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <cmsis_core.h>
#include <zephyr/sys/barrier.h>

#include <OsIf.h>

#define REG_CHECK_MASK(_reg, _mask, _val)                                                          \
	while ((*(volatile uint32_t *)(_reg) & (_mask)) != (_val)) {                               \
	}
#define REG_SET_MASK(_reg, _mask)                                                                  \
	{                                                                                          \
		*(volatile uint32_t *)(_reg) |= (_mask);                                           \
	}
#define REG_CLEAR_MASK(_reg, _mask)                                                                \
	{                                                                                          \
		*(volatile uint32_t *)(_reg) &= (~_mask);                                          \
	}
#ifndef REG_WRITE32
#define REG_WRITE32(_reg, _val)                                                                    \
	{                                                                                          \
		*(volatile uint32_t *)(_reg) = (_val);                                             \
	}
#endif // !defined REG_WRITE32

void ALWAYS_INLINE soc_partition_sram_init(uint32_t base_addr, uint32_t sz)
{
	// Clear previous init status, if any
	REG_WRITE32(base_addr + 0xc, 0x00000001);
	// Set initialization address register start & end
	REG_WRITE32(base_addr + 0x4, 0x00000000);
	REG_WRITE32(base_addr + 0x8, sz - 1);
	// Trigger the initialization: set INITREQ in PRAMCR, and wait for completion
	REG_WRITE32(base_addr + 0x0, 0x00000001);
	REG_CHECK_MASK(base_addr + 0xc, 0x00000001, 0x00000001);
}

void z_arm_platform_init(void)
{
	/* enable peripheral port access at EL1 and EL0 */
	__asm__ volatile("mrc p15, 0, r0, c15, c0, 0\n");
	__asm__ volatile("orr r0, #1\n");
	__asm__ volatile("mcr p15, 0, r0, c15, c0, 0\n");
	barrier_dsync_fence_full();
	barrier_isync_fence_full();

	/*
	 * Take exceptions in Arm mode because Zephyr ASM code for Cortex-R Aarch32
	 * is written for Arm
	 */
	__set_SCTLR(__get_SCTLR() & ~SCTLR_TE_Msk);

	if (IS_ENABLED(CONFIG_ICACHE)) {
		if (!(__get_SCTLR() & SCTLR_I_Msk)) {
			L1C_InvalidateICacheAll();
			__set_SCTLR(__get_SCTLR() | SCTLR_I_Msk);
			barrier_isync_fence_full();
		}
	}

	if (IS_ENABLED(CONFIG_DCACHE)) {
		if (!(__get_SCTLR() & SCTLR_C_Msk)) {
			L1C_InvalidateDCacheAll();
			__set_SCTLR(__get_SCTLR() | SCTLR_C_Msk);
			barrier_dsync_fence_full();
		}
	}

	/* Need to initialize the Data SRAM for RTU0 which isn't done by the HSE Boot Rom
	 * for some reason.
	 */
	// soc_partition_sram_init(0x761D0000, 0x40000);
	// soc_partition_sram_init(0x761E0000, 0x40000);
	// soc_partition_sram_init(0x761F0000, 0x800000);

	/* Initialize the stacks. It can't use the INIT_STACKS feature since that uses 0xAA and the
	 * compiler assumes 0 in uninit stack space for optimization -- I think.
	 * Access to an uninitialized SRAM address will ABORT in the R52. */
	extern char z_interrupt_stacks[];
	extern char z_arm_fiq_stack[];
	extern char z_arm_abort_stack[];
	extern char z_arm_undef_stack[];
	extern char z_arm_svc_stack[];
	extern char z_arm_sys_stack[];
	memset(z_arm_fiq_stack, 0, CONFIG_ARMV7_FIQ_STACK_SIZE);
	memset(z_interrupt_stacks, 0, CONFIG_ISR_STACK_SIZE);
	memset(z_arm_abort_stack, 0, CONFIG_ARMV7_EXCEPTION_STACK_SIZE);
	memset(z_arm_undef_stack, 0, CONFIG_ARMV7_EXCEPTION_STACK_SIZE);
	memset(z_arm_svc_stack, 0, CONFIG_ARMV7_SVC_STACK_SIZE);
	memset(z_arm_sys_stack, 0, CONFIG_ARMV7_SYS_STACK_SIZE);
}

static int soc_init(void)
{
	OsIf_Init(NULL);

	return 0;
}

// #ifdef CONFIG_CLOCK_CONTROL_NXP_S32
static int soc_platform_init(void)
{
	// TODO: This debugging enabler is likely not necessary
	*(volatile uint32_t *)0x4dc100c0 = 0x3cf3cf00;

	/* If this element is responsible for the clock configuration then the
	 * reset must include the RTU1 partition, otherwise multiple clock element
	 * configurations will fail.
	 */

	extern char _nocache_ram_start[];
	extern char _nocache_ram_end[];
	memset(&_nocache_ram_start, 0,
	       (uintptr_t)&_nocache_ram_end - (uintptr_t)&_nocache_ram_start);

	// Enable partition clocks and confirm
	REG_SET_MASK(0x41900500, 0x00000001);
	REG_SET_MASK(0x41900504, 0x00000001);
	REG_WRITE32(0x41900000, 0x5AF0);
	REG_WRITE32(0x41900000, 0xA50F);
	REG_CHECK_MASK(0x41900504, 0x00000001, 0x00000000);
	REG_CHECK_MASK(0x41900508, 0x00000001, 0x00000001);

	// Clear the forced reset from Periph128 for RTU1 partition
	REG_CLEAR_MASK(0x41850050, 0x00000001);

	// Disable partition output isolation and confirm
	REG_CLEAR_MASK(0x41900500, 0x00000004);
	REG_SET_MASK(0x41900504, 0x00000004);
	REG_WRITE32(0x41900000, 0x5AF0);
	REG_WRITE32(0x41900000, 0xA50F);
	REG_CHECK_MASK(0x41900504, 0x00000004, 0x00000000);
	REG_CHECK_MASK(0x41900508, 0x00000004, 0x00000000);

	// Confirm the partition is out of forced reset
	REG_CHECK_MASK(0x41850150, 0x00000001, 0x00000000);

	// Set the Fence & Drain controls
	REG_WRITE32(0x41860064, 0x0);

	// Enable the partition interconnect interface
	REG_SET_MASK(0x418A0004, 0x80000000);
	REG_CLEAR_MASK(0x418A0004, 0x00000008);
	REG_CHECK_MASK(0x418A0084, 0x00000010, 0x0);
	REG_CLEAR_MASK(0x418A0004, 0x80000000);

	// Enable the RTU1 NIC
	REG_WRITE32(0x75400000, 0x2);
	REG_WRITE32(0x75500000, 0x2);
	REG_WRITE32(0x75600000, 0x2);
	REG_WRITE32(0x75700000, 0x2);

	return 0;
}

SYS_INIT(soc_platform_init, EARLY, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
// #endif // CONFIG_CLOCK_CONTROL_NXP_S32

SYS_INIT(soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
