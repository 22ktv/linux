/*
 *  Copyright © 2014-2015 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VC5_REGS_H
#define VC5_REGS_H

#include <linux/bitops.h>

#define VC5_MASK(high, low) ((u32)GENMASK(high, low))
/* Using the GNU statement expression extension */
#define VC5_SET_FIELD(value, field)					\
	({								\
		uint32_t fieldval = (value) << field##_SHIFT;		\
		WARN_ON((fieldval & ~field##_MASK) != 0);		\
		fieldval & field##_MASK;				\
	 })

#define VC5_GET_FIELD(word, field) (((word) & field##_MASK) >>		\
				    field##_SHIFT)

#define V3D_HUB_AXICFG                                 0x00000
# define V3D_HUB_AXICFG_MAX_LEN_MASK                   VC5_MASK(3, 0)
# define V3D_HUB_AXICFG_MAX_LEN_SHIFT                  0
#define V3D_HUB_UIFCFG                                 0x00004
#define V3D_HUB_IDENT0                                 0x00008

#define V3D_HUB_IDENT1                                 0x0000c
# define V3D_HUB_IDENT1_WITH_MSO                       BIT(19)
# define V3D_HUB_IDENT1_WITH_TSY                       BIT(18)
# define V3D_HUB_IDENT1_WITH_TFU                       BIT(17)
# define V3D_HUB_IDENT1_WITH_L3C                       BIT(16)
# define V3D_HUB_IDENT1_NHOSTS_MASK                    VC5_MASK(15, 12)
# define V3D_HUB_IDENT1_NHOSTS_SHIFT                   12
# define V3D_HUB_IDENT1_NCORES_MASK                    VC5_MASK(11, 8)
# define V3D_HUB_IDENT1_NCORES_SHIFT                   8
# define V3D_HUB_IDENT1_REV_MASK                       VC5_MASK(7, 4)
# define V3D_HUB_IDENT1_REV_SHIFT                      4
# define V3D_HUB_IDENT1_TVER_MASK                      VC5_MASK(3, 0)
# define V3D_HUB_IDENT1_TVER_SHIFT                     0

#define V3D_HUB_IDENT2                                 0x00010
# define V3D_HUB_IDENT2_WITH_MMU                       BIT(8)
# define V3D_HUB_IDENT2_L3C_NKB_MASK                   VC5_MASK(7, 0)
# define V3D_HUB_IDENT2_L3C_NKB_SHIFT                  0

#define V3D_HUB_IDENT3                                 0x00014
# define V3D_HUB_IDENT3_IPREV_MASK                     VC5_MASK(15, 8)
# define V3D_HUB_IDENT3_IPREV_SHIFT                    8
# define V3D_HUB_IDENT3_IPIDX_MASK                     VC5_MASK(7, 0)
# define V3D_HUB_IDENT3_IPIDX_SHIFT                    0

#define V3D_HUB_INT_STS                                0x00050
#define V3D_HUB_INT_SET                                0x00054
#define V3D_HUB_INT_CLR                                0x00058
#define V3D_HUB_INT_MSK_STS                            0x0005c
#define V3D_HUB_INT_MSK_SET                            0x00060
#define V3D_HUB_INT_MSK_CLR                            0x00064
# define V3D_HUB_INT_MMU_WRV                           BIT(5)
# define V3D_HUB_INT_MMU_PTI                           BIT(4)
# define V3D_HUB_INT_MMU_CAP                           BIT(3)
# define V3D_HUB_INT_MSO                               BIT(2)
# define V3D_HUB_INT_TFUC                              BIT(1)
# define V3D_HUB_INT_TFUF                              BIT(0)

#define V3D_MMUC_CONTROL                               0x01000
# define V3D_MMUC_CONTROL_CLEAR                        BIT(3)
# define V3D_MMUC_CONTROL_FLUSHING                     BIT(2)
# define V3D_MMUC_CONTROL_FLUSH                        BIT(1)
# define V3D_MMUC_CONTROL_ENABLE                       BIT(0)

#define V3D_MMU_0_CTL                                  0x01200
#define V3D_MMU_1_CTL                                  0x01300
# define V3D_MMU_CTL_CAP_EXCEEDED                      BIT(27)
# define V3D_MMU_CTL_CAP_EXCEEDED_ABORT                BIT(26)
# define V3D_MMU_CTL_CAP_EXCEEDED_INT                  BIT(25)
# define V3D_MMU_CTL_CAP_EXCEEDED_EXCEPTION            BIT(24)
# define V3D_MMU_CTL_PT_INVALID                        BIT(20)
# define V3D_MMU_CTL_PT_INVALID_ABORT                  BIT(19)
# define V3D_MMU_CTL_PT_INVALID_INT                    BIT(18)
# define V3D_MMU_CTL_PT_INVALID_EXCEPTION              BIT(17)
# define V3D_MMU_CTL_WRITE_VIOLATION                   BIT(16)
# define V3D_MMU_CTL_WRITE_VIOLATION_ABORT             BIT(11)
# define V3D_MMU_CTL_WRITE_VIOLATION_INT               BIT(10)
# define V3D_MMU_CTL_WRITE_VIOLATION_EXCEPTION         BIT(9)
# define V3D_MMU_CTL_TLB_CLEARING                      BIT(7)
# define V3D_MMU_CTL_TLB_STATS_CLEAR                   BIT(3)
# define V3D_MMU_CTL_TLB_CLEAR                         BIT(2)
# define V3D_MMU_CTL_TLB_STATS_ENABLE                  BIT(1)
# define V3D_MMU_CTL_ENABLE                            BIT(0)

#define V3D_MMU_0_PT_PA_BASE                           0x01204
#define V3D_MMU_0_HIT                                  0x01208
#define V3D_MMU_0_MISSES                               0x0120c
#define V3D_MMU_0_STALLS                               0x01210

#define V3D_MMU_0_ADDR_CAP                             0x01214
# define V3D_MMU_ADDR_CAP_ENABLE                       BIT(31)
# define V3D_MMU_ADDR_CAP_MPAGE_MASK                   VC5_MASK(11, 0)
# define V3D_MMU_ADDR_CAP_MPAGE_SHIFT                  0

#define V3D_MMU_0_SHOOT_DOWN                           0x01218
# define V3D_MMU_SHOOT_DOWN_SHOOTING                   BIT(29)
# define V3D_MMU_SHOOT_DOWN_SHOOT                      BIT(28)
# define V3D_MMU_SHOOT_DOWN_PAGE_MASK                  VC5_MASK(27, 0)
# define V3D_MMU_SHOOT_DOWN_PAGE_SHIFT                 0

#define V3D_MMU_0_BYPASS_START                         0x0121c
#define V3D_MMU_0_BYPASS_END                           0x01220

/* AXI ID of the access that faulted */
#define V3D_MMU_0_VIO_ID                               0x0122c

/* Address for illegal PTEs to return */
#define V3D_MMU_0_ILLEGAL_ADDR                         0x01230

/* Address that faulted */
#define V3D_MMU_0_VIO_ADDR                             0x01234

#define V3D_GCA_CACHE_CTRL                             0x0410c
# define V3D_GCA_CACHE_CTRL_FLUSH                      BIT(0)

#define V3D_GCA_SAFE_SHUTDOWN                          0x041b0
# define V3D_GCA_SAFE_SHUTDOWN_EN                      BIT(0)

#define V3D_GCA_SAFE_SHUTDOWN_ACK                      0x041b4
# define V3D_GCA_SAFE_SHUTDOWN_ACK_ACKED               3

# define V3D_TOP_GR_BRIDGE_SW_INIT_0                   0x04008
# define V3D_TOP_GR_BRIDGE_SW_INIT_0_V3D_CLK_108_SW_INIT BIT(0)

#define V3D_CTL_0_IDENT0                               0x08000
#define V3D_CTL_1_IDENT0                               0x10000
# define V3D_IDENT0_VER_MASK                           VC5_MASK(31, 24)
# define V3D_IDENT0_VER_SHIFT                          24

#define V3D_CTL_0_IDENT1                               0x08004
/* Multiples of 1kb */
# define V3D_IDENT1_VPM_SIZE_MASK                      VC5_MASK(31, 28)
# define V3D_IDENT1_VPM_SIZE_SHIFT                     28
# define V3D_IDENT1_NSEM_MASK                          VC5_MASK(23, 16)
# define V3D_IDENT1_NSEM_SHIFT                         16
# define V3D_IDENT1_NTMU_MASK                          VC5_MASK(15, 12)
# define V3D_IDENT1_NTMU_SHIFT                         12
# define V3D_IDENT1_QUPS_MASK                          VC5_MASK(11, 8)
# define V3D_IDENT1_QUPS_SHIFT                         8
# define V3D_IDENT1_NSLC_MASK                          VC5_MASK(7, 4)
# define V3D_IDENT1_NSLC_SHIFT                         4
# define V3D_IDENT1_REV_MASK                           VC5_MASK(3, 0)
# define V3D_IDENT1_REV_SHIFT                          0

#define V3D_CTL_0_IDENT2                               0x08008
# define V3D_IDENT2_BCG_INT                            BIT(28)

#define V3D_CTL_0_MISCCFG                              0x08018
# define V3D_MISCCFG_OVRTMUOUT                         BIT(0)

#define V3D_CTL_0_L2CACTL                              0x08020
# define V3D_L2CACTL_L2CCLR                            BIT(2)
# define V3D_L2CACTL_L2CDIS                            BIT(1)
# define V3D_L2CACTL_L2CENA                            BIT(0)

#define V3D_CTL_0_SLCACTL                              0x08024
# define V3D_SLCACTL_TVCCS_MASK                        VC5_MASK(27, 24)
# define V3D_SLCACTL_TVCCS_SHIFT                       24
# define V3D_SLCACTL_TDCCS_MASK                        VC5_MASK(19, 16)
# define V3D_SLCACTL_TDCCS_SHIFT                       16
# define V3D_SLCACTL_UCC_MASK                          VC5_MASK(11, 8)
# define V3D_SLCACTL_UCC_SHIFT                         8
# define V3D_SLCACTL_ICC_MASK                          VC5_MASK(3, 0)
# define V3D_SLCACTL_ICC_SHIFT                         0

#define V3D_CTL_0_L2TCACTL                             0x08030
# define V3D_L2TCACTL_TMUWCF                           BIT(8)
# define V3D_L2TCACTL_L2T_NO_WM                        BIT(4)
# define V3D_L2TCACTL_FLM_FLUSH                        0
# define V3D_L2TCACTL_FLM_CLEAR                        1
# define V3D_L2TCACTL_FLM_CLEAN                        2
# define V3D_L2TCACTL_FLM_MASK                         VC5_MASK(2, 1)
# define V3D_L2TCACTL_FLM_SHIFT                        1
# define V3D_L2TCACTL_L2TFLS                           BIT(0)
#define V3D_CTL_0_L2TFLSTA                             0x08034
#define V3D_CTL_0_L2TFLEND                             0x08038

#define V3D_CTL_0_INT_STS                              0x08050
#define V3D_CTL_0_INT_SET                              0x08054
#define V3D_CTL_0_INT_CLR                              0x08058
#define V3D_CTL_0_INT_MSK_STS                          0x0805c
#define V3D_CTL_0_INT_MSK_SET                          0x08060
#define V3D_CTL_0_INT_MSK_CLR                          0x08064
# define V3D_INT_QPU_MASK                              VC5_MASK(27, 16)
# define V3D_INT_QPU_SHIFT                             16
# define V3D_INT_GMPV                                  BIT(5)
# define V3D_INT_TRFB                                  BIT(4)
# define V3D_INT_SPILLUSE                              BIT(3)
# define V3D_INT_OUTOMEM                               BIT(2)
# define V3D_INT_FLDONE                                BIT(1)
# define V3D_INT_FRDONE                                BIT(0)

#define V3D_CLE_0_CT0CS                                0x08100
#define V3D_CLE_0_CT1CS                                0x08104
#define V3D_CLE_0_CTNCS(n) (V3D_CLE_0_CT0CS + 4 * n)
#define V3D_CLE_0_CT0EA                                0x08108
#define V3D_CLE_0_CT1EA                                0x0810c
#define V3D_CLE_0_CTNEA(n) (V3D_CLE_0_CT0EA + 4 * n)
#define V3D_CLE_0_CT0CA                                0x08110
#define V3D_CLE_0_CT1CA                                0x08114
#define V3D_CLE_0_CTNCA(n) (V3D_CLE_0_CT0CA + 4 * n)
#define V3D_CLE_0_CT0RA                                0x08118
#define V3D_CLE_0_CT1RA                                0x0811c
#define V3D_CLE_0_CT0LC                                0x08120
#define V3D_CLE_0_CT1LC                                0x08124
#define V3D_CLE_0_CT0PC                                0x08128
#define V3D_CLE_0_CT1PC                                0x0812c
#define V3D_CLE_0_PCS                                  0x08130
#define V3D_CLE_0_BFC                                  0x08134
#define V3D_CLE_0_RFC                                  0x08138
#define V3D_CLE_0_TFBC                                 0x0813c
#define V3D_CLE_0_TFIT                                 0x08140
#define V3D_CLE_0_CT1CFG                               0x08144
#define V3D_CLE_0_CT1TILECT                            0x08148
#define V3D_CLE_0_CT1TSKIP                             0x0814c
#define V3D_CLE_0_CT1PTCT                              0x08150
#define V3D_CLE_0_CT0SYNC                              0x08154
#define V3D_CLE_0_CT1SYNC                              0x08158
#define V3D_CLE_0_CT0QBA                               0x08160
#define V3D_CLE_0_CT1QBA                               0x08164
#define V3D_CLE_0_CTNQBA(n) (V3D_CLE_0_CT0QBA + 4 * n)
#define V3D_CLE_0_CT0QEA                               0x08168
#define V3D_CLE_0_CT1QEA                               0x0816c
#define V3D_CLE_0_CTNQEA(n) (V3D_CLE_0_CT0QEA + 4 * n)
#define V3D_CLE_0_CT0QMA                               0x08170
#define V3D_CLE_0_CT0QMS                               0x08174
#define V3D_CLE_0_CT1QCFG                              0x08178
/* If set without ETPROC, entirely skip tiles with no primitives. */
# define V3D_CLE_QCFG_ETFILT                           BIT(7)
/* If set with ETFILT, just write the clear color to tiles with no
 * primitives.
 */
# define V3D_CLE_QCFG_ETPROC                           BIT(6)
# define V3D_CLE_QCFG_ETSFLUSH                         BIT(1)
# define V3D_CLE_QCFG_MCDIS                            BIT(0)

#define V3D_PTB_0_BPCA                                 0x08300
#define V3D_PTB_0_BPCS                                 0x08304
#define V3D_PTB_0_BPOA                                 0x08308
#define V3D_PTB_0_BPOS                                 0x0830c

#define V3D_PTB_0_BXCF                                 0x08310
# define V3D_PTB_BXCF_RWORDERDISA                      BIT(1)
# define V3D_PTB_BXCF_CLIPDISA                         BIT(0)

#define V3D_GMP_0_STATUS                               0x08800
# define V3D_GMP_STATUS_GMPRST                         BIT(31)
# define V3D_GMP_STATUS_WR_COUNT_MASK                  VC5_MASK(30, 24)
# define V3D_GMP_STATUS_WR_COUNT_SHIFT                 24
# define V3D_GMP_STATUS_RD_COUNT_MASK                  VC5_MASK(22, 16)
# define V3D_GMP_STATUS_RD_COUNT_SHIFT                 16
# define V3D_GMP_STATUS_WR_ACTIVE                      BIT(5)
# define V3D_GMP_STATUS_RD_ACTIVE                      BIT(4)
# define V3D_GMP_STATUS_CFG_BUSY                       BIT(3)
# define V3D_GMP_STATUS_CNTOVF                         BIT(2)
# define V3D_GMP_STATUS_INVPROT                        BIT(1)
# define V3D_GMP_STATUS_VIO                            BIT(0)

#define V3D_GMP_0_CFG                                  0x08804
# define V3D_GMP_CFG_LBURSTEN                          BIT(3)
# define V3D_GMP_CFG_PGCRSEN                           BIT()
# define V3D_GMP_CFG_STOP_REQ                          BIT(1)
# define V3D_GMP_CFG_PROT_ENABLE                       BIT(0)

#define V3D_GMP_0_VIO_ADDR                             0x08808
#define V3D_GMP_0_VIO_TYPE                             0x0880c
#define V3D_GMP_0_TABLE_ADDR                           0x08810
#define V3D_GMP_0_CLEAR_LOAD                           0x08814
#define V3D_GMP_0_PRESERVE_LOAD                        0x08818
#define V3D_GMP_0_VALID_LINES                          0x08820

/* Each core's V3D_CTL is offset by this far in the address space. */
#define V3D_CTL_CORE_OFFSET    (V3D_CTL_1_IDENT0 - V3D_CTL_0_IDENT0)
/* Each core's V3D_CTL is offset by this far in the address space. */
#define V3D_MMU_CORE_OFFSET    (V3D_MMU_1_CTRL - V3D_MMU_0_CTRL)

#endif /* VC5_REGS_H */
