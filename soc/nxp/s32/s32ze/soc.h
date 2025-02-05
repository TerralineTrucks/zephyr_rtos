/*
 * Copyright 2022-2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NXP_S32_S32ZE_SOC_H_
#define _NXP_S32_S32ZE_SOC_H_

/* Do not let CMSIS to handle GIC */
#define __GIC_PRESENT 0

#if defined(CONFIG_SOC_S32Z270)
#include <S32Z2.h>
#else
#error "SoC not supported"
#endif

#if defined(CONFIG_CMSIS_RTOS_V2)
#include <cmsis_rtos_v2_adapt.h>
#endif

/* Aliases for peripheral base addresses */

/* SIUL2 */
#define IP_SIUL2_2_BASE         0U  /* instance does not exist on this SoC */

/* LINFlexD*/
#define IP_LINFLEX_12_BASE      IP_MSC_0_LIN_BASE

/* SWT */
#define IP_SWT_0_BASE           IP_CE_SWT_0_BASE
#define IP_SWT_1_BASE           IP_CE_SWT_1_BASE
#define IP_SWT_2_BASE           IP_RTU0__SWT_0_BASE
#define IP_SWT_3_BASE           IP_RTU0__SWT_1_BASE
#define IP_SWT_4_BASE           IP_RTU0__SWT_2_BASE
#define IP_SWT_5_BASE           IP_RTU0__SWT_3_BASE
#define IP_SWT_6_BASE           IP_RTU0__SWT_4_BASE
#define IP_SWT_7_BASE           IP_RTU1__SWT_0_BASE
#define IP_SWT_8_BASE           IP_RTU1__SWT_1_BASE
#define IP_SWT_9_BASE           IP_RTU1__SWT_2_BASE
#define IP_SWT_10_BASE          IP_RTU1__SWT_3_BASE
#define IP_SWT_11_BASE          IP_RTU1__SWT_4_BASE
#define IP_SWT_12_BASE          IP_SMU__SWT_BASE

/* STM */
#define IP_STM_0_BASE           IP_CE_STM_0_BASE
#define IP_STM_1_BASE           IP_CE_STM_1_BASE
#define IP_STM_2_BASE           IP_CE_STM_2_BASE
#define IP_STM_3_BASE           IP_RTU0__STM_0_BASE
#define IP_STM_4_BASE           IP_RTU0__STM_1_BASE
#define IP_STM_5_BASE           IP_RTU0__STM_2_BASE
#define IP_STM_6_BASE           IP_RTU0__STM_3_BASE
#define IP_STM_7_BASE           IP_RTU1__STM_0_BASE
#define IP_STM_8_BASE           IP_RTU1__STM_1_BASE
#define IP_STM_9_BASE           IP_RTU1__STM_2_BASE
#define IP_STM_10_BASE          IP_RTU1__STM_3_BASE
#define IP_STM_11_BASE          IP_SMU__STM_0_BASE
#define IP_STM_12_BASE          IP_SMU__STM_2_BASE

/* NETC */
#define IP_NETC_EMDIO_0_BASE    IP_NETC__EMDIO_BASE_BASE

/* MRU */
#define IP_MRU_0_BASE           IP_RTU0__MRU_0_BASE
#define IP_MRU_1_BASE           IP_RTU0__MRU_1_BASE
#define IP_MRU_2_BASE           IP_RTU0__MRU_2_BASE
#define IP_MRU_3_BASE           IP_RTU0__MRU_3_BASE
#define IP_MRU_4_BASE           IP_RTU1__MRU_0_BASE
#define IP_MRU_5_BASE           IP_RTU1__MRU_1_BASE
#define IP_MRU_6_BASE           IP_RTU1__MRU_2_BASE
#define IP_MRU_7_BASE           IP_RTU1__MRU_3_BASE

/* ME */
#define MC_ME_BASE          0x41900000
#define ME_SMU_OFFSET       0x100
#define ME_RTU0_OFFSET      0x300
#define ME_RTU1_OFFSET      0x500
#define ME_CORE0_OFFSET     0x40
#define ME_CORE1_OFFSET     0x60
#define ME_CORE2_OFFSET     0x80
#define ME_CORE3_OFFSET     0xa0
#define ME_CORE_CONF_OFFSET 0x0
#define ME_CORE_UPD_OFFSET  0x4
#define ME_CORE_STAT_OFFSET 0x8
#define ME_CORE_ADDR_OFFSET 0xc

/* RGM */
#define MC_RGM_BASE                0x41850000
#define RGM_RTU0_OFFSET            0x48
#define RGM_RTU1_OFFSET            0x50
#define RGM_SUBSYS_RESET_MASK      0x1
/* Periph 64 is the whole RTU0 subsystem
 * Periph 65-68 in PRST1 are RTU0 Core 0-3
 * Periph 128 is the whole RTU1 subsystem
 * Periph 129-132 in PRST2 are RTU1 Core 0-3 */
#define RGM_CORE_RESET_MASK(_core) (1 << ((_core) + 1))

/* RTU GPR */
#define RTU0_GPR_BASE                  0x76120000
#define RTU1_GPR_BASE                  0x76920000
#define RTU_GPR_CORE_CONF_OFFSET       0x0
#define RTU_GPR_CONF_THUMB_MASK        (1 << 2)
#define RTU_GPR_HALT_OFFSET            0x14
#define RTU_GPR_HALT_MASK(_core)       (1 << (_core))
#define RTU_GPR_ISOLATION_OFFSET       0xC
#define RTU_GPR_ISOLATION_MASK(_core)  (0x10000111 << (_core))
#define RTU_GPR_ISO_STATUS_OFFSET      0x1C
#define RTU_GPR_ISO_STATUS_MASK(_core) (0x111 << (_core))

#endif /* _NXP_S32_S32ZE_SOC_H_ */
