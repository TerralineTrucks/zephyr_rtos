/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
	// If using local addresses, it is 8-byte shifted, but should be the last one to program
	REG_WRITE32(base_addr + 0x8, (sz >> 3) - 1);

	// Trigger the initialization: set INITREQ in PRAMCR, and wait for completion
	REG_WRITE32(base_addr + 0x0, 0x00000001);
	REG_CHECK_MASK(base_addr + 0xc, 0x00000001, 0x00000001);
}

void z_arm_platform_init(void)
{
#ifdef CONFIG_MCUBOOT
/* For booting, this should be configured to use stack located in
 * Rtu0 Core0 TCMs. Configure the IMP_*TCMREGIONR for the core to use them.
 * TCM A = 64 KB, has 1 wait state, 0x30000000
 * TCM B = 16 KB, has 0 wait state, 0x30100000
 * TCM C = 16 KB, has 1 wait state, 0x30200000
 */
#define RTU0_CORE0_TCMBASE  0x30000000
#define TCMA_OFFSET         0x0
#define TCMB_OFFSET         0x100000
#define TCMC_OFFSET         0x200000
#define TCMREGION_SIZE_16KB 0b00101
#define TCMREGION_SIZE_64KB 0b00111

#define TCM_INIT_BASE CONFIG_SRAM_BASE_ADDRESS
#define TCM_OP_COUNT  (CONFIG_SRAM_SIZE * 1024 / 8)

	/* Configure TCM A */
	uint32_t val = RTU0_CORE0_TCMBASE + TCMA_OFFSET;
	val |= TCMREGION_SIZE_64KB << 2;
	val |= 1 << 8; // Wait States (1/0)
	val |= 1 << 1; // Enable (1), Disable (0) for EL2
	val |= 1 << 0; // Enable (1), Disable (0) for EL1 & EL0
	__set_CP(15, 0, val, 9, 1, 0);

	/* Configure TCM B */
	val = RTU0_CORE0_TCMBASE + TCMB_OFFSET;
	val |= TCMREGION_SIZE_16KB << 2;
	val |= 0 << 8; // Wait States (1/0)
	val |= 1 << 1; // Enable (1), Disable (0) for EL2
	val |= 1 << 0; // Enable (1), Disable (0) for EL1 & EL0
	__set_CP(15, 0, val, 9, 1, 1);

	/* Configure TCM C */
	val = RTU0_CORE0_TCMBASE + TCMC_OFFSET;
	val |= TCMREGION_SIZE_16KB << 2;
	val |= 1 << 8; // Wait States (1/0)
	val |= 1 << 1; // Enable (1), Disable (0) for EL2
	val |= 1 << 0; // Enable (1), Disable (0) for EL1 & EL0
	__set_CP(15, 0, val, 9, 1, 2);

	/* This is intrinsically going to use stack which may not have
	 * been initialized yet depending on hardware configuration.
	 * Disable RAM ECC so the partial line loads for stack values to complete
	 * the full line memory writes won't trap on access.
	 */
	uint32_t flash_prot_en = 1;
	uint32_t ram_prot_en = 0;
	__set_CP(15, 1, ((flash_prot_en << 1) | (ram_prot_en << 0)), 9, 1, 2);

	/* Zero out the TCMA.
	 * If the SRAM_BASE was moved outside of the core-local memories then this will
	 * trap & abort on random SRAM partial loads with ECC failures (from non-initialized
	 * system SRAM).
	 *
	 * Do this in QuadWord store operations to ensure the complete ECC line
	 * fills at the same time to avoid partial RMW and faults at the bus.
	 */
	__asm__ volatile("qword_copy_init:			\t\n"
			 "	mov r1, %0			\t\n"
			 "	mov r3, %1			\t\n"
			 "	mov r4, #0			\t\n"
			 "	mov r5, #0			\t\n"
			 "qword_copy:			\t\n"
			 "	strd r4, r5, [r1]	\t\n"
			 "  ldrd r4, r5, [r1]	\t\n"
			 "  add r1, r1, #8		\t\n"
			 "	sub r3, r3, #1		\t\n"
			 "  cmp r3, #0			\t\n"
			 "	bne qword_copy		\t\n"
			 :
			 : "I"(TCM_INIT_BASE), "I"(TCM_OP_COUNT)
			 : "memory");

	/* Re-enable the TCM & Cache ECC protection now. */
	ram_prot_en = 1;
	__set_CP(15, 1, ((flash_prot_en << 1) | (ram_prot_en << 0)), 9, 1, 2);
#endif /* CONFIG_MCUBOOT */

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
	} else {
		/* Explicitly disable the ICACHE */
		__set_SCTLR(__get_SCTLR() & (~SCTLR_I_Msk));
		__ISB();
	}

	if (IS_ENABLED(CONFIG_DCACHE)) {
		if (!(__get_SCTLR() & SCTLR_C_Msk)) {
			L1C_InvalidateDCacheAll();
			__set_SCTLR(__get_SCTLR() | SCTLR_C_Msk);
			barrier_dsync_fence_full();
		}
	} else {
		/* Explicitly disable the DCACHE */
		__set_SCTLR(__get_SCTLR() & (~SCTLR_C_Msk));
		__ISB();
	}
}

static int soc_init(void)
{
	OsIf_Init(NULL);

	return 0;
}

static int soc_rtu_start_cores(int rtu_index)
{
	uint32_t current_core = MPIDR_TO_CORE(GET_MPIDR());

	const uint32_t rtu_gpr_map[2] = {RTU0_GPR_BASE, RTU1_GPR_BASE};
	const uint32_t rgm_rtu_map[2] = {RGM_RTU0_OFFSET, RGM_RTU1_OFFSET};
	const uint32_t me_rtu_map[2] = {ME_RTU0_OFFSET, ME_RTU1_OFFSET};
	const uint32_t me_core_map[4] = {ME_CORE0_OFFSET, ME_CORE1_OFFSET, ME_CORE2_OFFSET,
					 ME_CORE3_OFFSET};

	uint32_t me_rtu_reg_base = MC_ME_BASE + me_rtu_map[rtu_index];
	uint32_t rgm_rtu_reg_base = MC_RGM_BASE + rgm_rtu_map[rtu_index];

	uint32_t rtu_gpr_conf = rtu_gpr_map[rtu_index] + RTU_GPR_CORE_CONF_OFFSET;
	uint32_t rtu_gpr_halt = rtu_gpr_map[rtu_index] + RTU_GPR_HALT_OFFSET;
	uint32_t rtu_gpr_isolation = rtu_gpr_map[rtu_index] + RTU_GPR_ISOLATION_OFFSET;
	uint32_t rtu_gpr_iso_status = rtu_gpr_map[rtu_index] + RTU_GPR_ISO_STATUS_OFFSET;

	/* Only the primary can initiate the other cores. */
	if (current_core != 0) {
		return 0;
	}

	extern void __start(void);
	extern void __start_t32(void);
	uint32_t start_addr = (uint32_t)__start;
	uint32_t rtu_conf = *(uint32_t *)rtu_gpr_conf;
	if ((rtu_conf & RTU_GPR_CONF_THUMB_MASK) != 0) {
		start_addr = (uint32_t)__start_t32;
	}

	/* CPU 0 is always the primary, skip it */
	for (int i = 1; i < arch_num_cpus(); i++) {
		uint32_t me_core_reg_base = me_rtu_reg_base + me_core_map[i];
		uint32_t me_core_conf = me_core_reg_base + ME_CORE_CONF_OFFSET;
		uint32_t me_core_upd = me_core_reg_base + ME_CORE_UPD_OFFSET;
		uint32_t me_core_stat = me_core_reg_base + ME_CORE_STAT_OFFSET;
		uint32_t me_core_addr = me_core_reg_base + ME_CORE_ADDR_OFFSET;
		uint32_t rgm_core_reset = rgm_rtu_reg_base;
		uint32_t rgm_reset_mask = RGM_CORE_RESET_MASK(i);

		/* Isolate the core */
		REG_SET_MASK(rtu_gpr_isolation, RTU_GPR_ISOLATION_MASK(i));
		REG_CHECK_MASK(rtu_gpr_iso_status, RTU_GPR_ISO_STATUS_MASK(i),
			       RTU_GPR_ISO_STATUS_MASK(i));

		/* Hold the core in reset */
		REG_SET_MASK(rgm_core_reset, rgm_reset_mask);

		/* Set the halt bit for the core */
		REG_SET_MASK(rtu_gpr_halt, RTU_GPR_HALT_MASK(i));

		/* Configure the boot address */
		REG_WRITE32(me_core_addr, start_addr);

		/* Release the core reset */
		REG_CLEAR_MASK(rgm_core_reset, rgm_reset_mask);
		REG_CHECK_MASK(rgm_core_reset, rgm_reset_mask, 0x0);

		/* Enable the clock */
		REG_WRITE32(me_core_conf, 0x1);
		REG_SET_MASK(me_core_upd, 0x1);
		REG_WRITE32(MC_ME_BASE, 0x5AF0);
		REG_WRITE32(MC_ME_BASE, 0xA50F);

		/* Confirm the values */
		REG_CHECK_MASK(me_core_upd, 0x1, 0x0);
		REG_CHECK_MASK(me_core_stat, 0x1, 0x1);

		/* Remove the core isolation hold */
		REG_CLEAR_MASK(rtu_gpr_isolation, RTU_GPR_ISOLATION_MASK(i));
		REG_CHECK_MASK(rtu_gpr_iso_status, RTU_GPR_ISO_STATUS_MASK(i), 0x0);

		/* Clear the halt bit for the core */
		REG_CLEAR_MASK(rtu_gpr_halt, RTU_GPR_HALT_MASK(i));
	}
	return 0;
}

static int soc_platform_init(void)
{
#ifdef CONFIG_MCUBOOT
#ifdef CONFIG_CLOCK_CONTROL_NXP_S32
	/* If this element is responsible for the clock configuration then the
	 * reset must include the RTU1 partition, otherwise multiple clock element
	 * configurations will fail.
	 */

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
#endif /* CONFIG_CLOCK_CONTROL_NXP_S32 */

	/* Perform ECC Initialization on the RTU 0 Data SRAM */
	soc_partition_sram_init(0x761D0000, 0x40000);
	soc_partition_sram_init(0x761E0000, 0x40000);
	soc_partition_sram_init(0x761F0000, 0x80000);

	/* Perform ECC Initialization on the remaining RTU 0 Code SRAM */
	// soc_partition_sram_init(0x760C0000, 0x100000);
	soc_partition_sram_init(0x760D0000, 0x100000);
	soc_partition_sram_init(0x760E0000, 0x100000);
	soc_partition_sram_init(0x760F0000, 0x100000);
	soc_partition_sram_init(0x76240000, 0x100000);
	soc_partition_sram_init(0x76250000, 0x100000);
	soc_partition_sram_init(0x76260000, 0x100000);

#if 0 // TODO: Debug and configure this for RTU1 use.
	/* Perform ECC Initialization on the RTU 1 Data SRAM */
	soc_partition_sram_init(0x769D0000, 0x40000);
	soc_partition_sram_init(0x769E0000, 0x40000);
	soc_partition_sram_init(0x769F0000, 0x80000);

	/* Perform ECC Initialization on the RTU 1 Code SRAM */
	soc_partition_sram_init(0x768C0000, 0x100000);
	soc_partition_sram_init(0x768D0000, 0x100000);
	soc_partition_sram_init(0x768E0000, 0x100000);
	soc_partition_sram_init(0x768F0000, 0x100000);
	soc_partition_sram_init(0x76A40000, 0x100000);
	soc_partition_sram_init(0x76A50000, 0x100000);
	soc_partition_sram_init(0x76A60000, 0x100000);
#endif
#else /* CONFIG_MCUBOOT */
#if CONFIG_MP_MAX_NUM_CPUS > 1
	return soc_rtu_start_cores(CONFIG_NXP_S32_RTU_INDEX);
#endif
#endif /* CONFIG_MCUBOOT */
	return 0;
}

SYS_INIT(soc_platform_init, EARLY, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

SYS_INIT(soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
