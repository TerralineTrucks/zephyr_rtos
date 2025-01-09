/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/linker/linker-defs.h>
#include <zephyr/arch/arm/mpu/arm_mpu.h>

#define DEVICE_REGION_START 0x40000000UL
#define DEVICE_REGION_END   0x76FFFFFFUL

#ifdef CONFIG_MEMC_NXP_S32_QSPI
#define QSPI0_AHB_REGION_START 0x00000000UL
#define QSPI0_AHB_REGION_END   0x07FFFFFFUL
#define QSPI1_AHB_REGION_START 0x10000000UL
#define QSPI1_AHB_REGION_END   0x17FFFFFFUL
#endif // CONFIG_MEMC_NXP_S32_QSPI

#ifdef CONFIG_MCUBOOT
#define CODE_SRAM0_REGION_START 0x32100000UL
#define CODE_SRAM0_REGION_END   0x327FFFFFUL
#define CODE_SRAM1_REGION_START 0x36100000UL
#define CODE_SRAM1_REGION_END   0x367fffffUL
#endif

static const struct arm_mpu_region mpu_regions[] = {
#ifdef CONFIG_MCUBOOT
	MPU_REGION_ENTRY("CODE_SRAM0", (uintptr_t)CODE_SRAM0_REGION_START,
			 REGION_FLASH_ATTR(CODE_SRAM0_REGION_END)),

	MPU_REGION_ENTRY("CODE_SRAM1", (uintptr_t)CODE_SRAM1_REGION_START,
			 REGION_FLASH_ATTR(CODE_SRAM1_REGION_END)),
#else
	MPU_REGION_ENTRY("vector", (uintptr_t)_vector_start,
			 REGION_RAM_TEXT_ATTR((uintptr_t)_vector_end)),

	MPU_REGION_ENTRY("SRAM_TEXT", (uintptr_t)__text_region_start,
			 REGION_RAM_TEXT_ATTR((uintptr_t)__rodata_region_start)),

	MPU_REGION_ENTRY("SRAM_RODATA", (uintptr_t)__rodata_region_start,
			 REGION_RAM_RO_ATTR((uintptr_t)__rodata_region_end)),
#endif // CONFIG_MCUBOOT

	MPU_REGION_ENTRY("SRAM_DATA", (uintptr_t)_image_ram_start,
			 REGION_RAM_ATTR((uintptr_t)__kernel_ram_end)),

	MPU_REGION_ENTRY("DEVICE", DEVICE_REGION_START, REGION_DEVICE_ATTR(DEVICE_REGION_END)),

#ifdef CONFIG_MEMC_NXP_S32_QSPI
	MPU_REGION_ENTRY("QSPI", (uintptr_t)QSPI0_AHB_REGION_START,
			 REGION_FLASH_ATTR(QSPI1_AHB_REGION_END)),
#endif // CONFIG_MEMC_NXP_S32_QSPI
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
