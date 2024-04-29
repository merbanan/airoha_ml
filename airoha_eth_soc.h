// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 *   Copyright (C) 2024 Benjamin Larsson <benjamin.larsson@genexis.eu>
 *
 *   Heavily based on mtk_eth_soc.h
 */

#ifndef AIROHA_ETH_H
#define AIROHA_ETH_H

#include <linux/dma-mapping.h>
#include <linux/netdevice.h>
#include <linux/of_net.h>
#include <linux/u64_stats_sync.h>
#include <linux/refcount.h>
#include <linux/phylink.h>
#include <linux/rhashtable.h>
#include <linux/dim.h>
#include <linux/bitfield.h>
#include <net/page_pool.h>
#include <linux/bpf_trace.h>

#define DBG_LB       (1 << 18)
#define DBG_HW       (1 << 19)
#define DBG_MSG      (1 << 20)
#define DBG_DES      (1 << 21)
#define DBG_REG      (1 << 22)
#define DBG_STAT     (1 << 23)
#define DBG_PNP      (1 << 24)
#define DBG_TIME     (1 << 25)
#define DBG_DBG      (1 << 26)
#define DBG_OUT      (1 << 27)
#define DBG_BUF      (1 << 28)
#define DBG_BUFS     (1 << 28)
#define DBG_FUNCTION (1 << 29)
#define DBG_FUNC     (1 << 29)
#define DBG_WARN     (1 << 30)
#define DBG_WARNING  (1 << 30)
#define DBG_ERROR    (1 << 31)
#define DBG_ERR      (1 << 31)


#define FE_BASE     		(0x1FB50000)
#define PSE_OQ_RSV     		(FE_BASE + 0x0090)

#define TCSUPPORT_CPU_EN7523 1

/* RSTCTRL2 */
#define QDMA1_RST			(1<<1)
#define QDMA2_RST			(1<<2)
#define FE_RST				(1<<21)
#define ESW_RST				(1<<23)


#define GDMA1_BASE(base)     		(base)
#define GDMA1_FWD_CFG(base)       ((base) + 0x00)
#define GDMA1_SHRP_CFG(base)      ((base) + 0x04)
#define GDMA1_MAC_ADRL(base)      ((base) + 0x08)
#define GDMA1_MAC_ADRH(base)      ((base) + 0x0c)
#define GDMA1_VLAN_GEN(base)      ((base) + 0x10)
#define GDMA1_LEN_CFG(base)       ((base) + 0x14)

/* define GDMA port */
#define GDM_P_PDMA						(0x0)
#define GDM_P_GDMA1						(0x1)
#define GDM_P_GDMA2						(0x2)
#define GDM_P_PPE						(0x4)
#define GDM_P_QDMA						(0x5)
#define GDM_P_DISCARD					(0x7)
#define GDM_P_CPU						 GDM_P_PDMA


#define AIROHA_MAX_DEVS	5

#define QDMA_CHANNELS		8
#define QDMA_QUEUE		8
#define HWFWD_DSCP_NUM		(32)
#define HWFWD_DSCP_SIZE		(16)
#define CONFIG_MAX_PKT_LENS					(2048)

#define DSCP_NUM_MAX		(4096)
#define IRQ_NUM_MAX		(4095)
#define TX0_DSCP_NUM		(16)
#define RX0_DSCP_NUM		(16)
#define TX0_DSCP_SIZE		(32)	//4 //4desc + 4msg
#define RX0_DSCP_SIZE		(32)	//4desc + 4msg

#define IRQ_DEPTH		(32)
#define MAX_PKT_LENS		(2048)
#define HWFWD_LOW_THRESHOLD		(1)
#define QDMA_RX_DSCP_MSG_LENS		(16)
#define QDMA_TX_DSCP_MSG_LENS		(8)

#define QDMA_TX_THRESHOLD		(4)
#define DESC_INFONODE_SIZE		(16)
#define DESC_INFO_SIZE			((TX0_DSCP_NUM + RX0_DSCP_NUM)*DESC_INFONODE_SIZE)
#define DESC_TOTAL_SIZE			(TX0_DSCP_NUM*TX0_DSCP_SIZE + RX0_DSCP_NUM*RX0_DSCP_SIZE)
#define HWFWD_TOTAL_SIZE		(MAX_PKT_LENS + HWFWD_DSCP_SIZE)* HWFWD_DSCP_NUM
#define MEM_POOL_SIZE			(DESC_TOTAL_SIZE + (IRQ_DEPTH<<2) + HWFWD_TOTAL_SIZE)
#define RING_IDX_0			0

/* GSW_CFG_GPC */
#define RX_CLK_MODE						(1<<2)

#define IRQ_CFG_RINGIDX_SHIFT						(16)
#define IRQ_CFG_RINGIDX_MASK						(0x1<<IRQ_CFG_RINGIDX_SHIFT)
#define IRQ_CFG_IDX_MASK							0xFFF

#define QDMA_ERR(B)			printk(B);
#define QDMA_LOG(B)			printk(B);
#define QDMA_MSG(...)			printk(__VA_ARGS__);


#define QDMA_ENABLE 1
#define QDMA_DISABLE 0
#define CONFIG_IRQ_DEF_VALUE			(0xFFFFFFFF)

typedef enum{
	QDMA_INT1_ENABLE1=1,
	QDMA_INT1_ENABLE2=2,
	QDMA_INT2_ENABLE1=3,
	QDMA_INT2_ENABLE2=4,
	QDMA_INT3_ENABLE1=5,
	QDMA_INT3_ENABLE2=6,
	QDMA_INT4_ENABLE1=7,
	QDMA_INT4_ENABLE2=8,
}QDMA_InterruptIdx_T;

#define RING_OFFSET(idx)                            (idx<<5)
#define RX_RING_NUM		                            (16)


#ifndef VPint
#define VPint			*(volatile unsigned long int *)
#endif
#ifndef VPshort
#define VPshort			*(volatile unsigned short *)
#endif
#ifndef VPchar
#define VPchar			*(volatile unsigned char *)
#endif

#define read_reg_word(reg) 				VPint(reg)
#define write_reg_word(reg, wdata) 		VPint(reg)=wdata


static inline u32 regRead32(u32 reg)		\
{						  	\
	return VPint(reg);			  	\
}		
static inline void regWrite32(u32 reg, u32 vlaue)	\
{                                                	\
        VPint(reg) = vlaue;                      	\
}


#define IO_GREG(reg)							read_reg_word((reg))
#define IO_SREG(reg, value)						write_reg_word((reg), value)
#define IO_GMASK(reg, mask, shift)				((read_reg_word((reg)) & mask) >> shift)
#define IO_SMASK(reg, mask, shift, value)		{ uint t = read_reg_word((reg)); write_reg_word((reg), ((t&~(mask))|((value<<shift)&mask))); }
#define IO_SBITS(reg, bit)						{ uint t = read_reg_word((reg)); write_reg_word((reg), (t|bit)); }
#define IO_CBITS(reg, bit)						{ uint t = regRead32(reg); regWrite32(reg, (t&~(bit))); }

/*******************************************************
 CSR for QDMA
********************************************************/
#define QDMA_CSR_INFO(base)							(base+0x0000)
#define QDMA_CSR_GLB_CFG(base)						(base+0x0004)

#define QDMA_CSR_HWFWD_DSCP_BASE(base)				(base+0x0010)
#define QDMA_CSR_HWFWD_BUFF_BASE(base)				(base+0x0014)
#define QDMA_CSR_HWFWD_DSCP_CFG(base)				(base+0x0018)

#define QDMA_CSR_INT_STATUS1(base)					(base+0x0020)
#define QDMA_CSR_INT_STATUS2(base)					(base+0x0024)
/*idx: 1~4*/
#define QDMA_CSR_INT_ENABLE1(base,idx)				(base+0x0020+((idx)<<3))
#define QDMA_CSR_INT_ENABLE2(base,idx)				(base+0x0024+((idx)<<3))

#define QDMA_CSR_IRQ_BASE(base)						(base+0x0050)
#define QDMA_CSR_IRQ_CFG(base)						(base+0x0054)
#define QDMA_CSR_IRQ_CLEAR_LEN(base)				(base+0x0058)
#define QDMA_CSR_IRQ_STATUS(base)					(base+0x005C)
#define QDMA_CSR_IRQ_PTIME(base)					(base+0x0060)

#define QDMA_CSR_TX_DSCP_BASE(base, idx)			(base+0x0100+RING_OFFSET(idx))
#define QDMA_CSR_TX_BLOCKING(base, idx)				(base+0x0104+RING_OFFSET(idx))
#define QDMA_CSR_TX_CPU_IDX(base, idx)				(base+0x0108+RING_OFFSET(idx))
#define QDMA_CSR_TX_DMA_IDX(base, idx)				(base+0x010C+RING_OFFSET(idx))

#define QDMA_CSR_RX_DSCP_BASE(base, idx)			(base+0x0200+RING_OFFSET(idx))
#define QDMA_CSR_RX_RING_SIZE(base, idx)			(base+0x0204+RING_OFFSET(idx))
#define QDMA_CSR_RX_CPU_IDX(base, idx)				(base+0x0208+RING_OFFSET(idx))
#define QDMA_CSR_RX_DMA_IDX(base, idx)				(base+0x020C+RING_OFFSET(idx))
#define QDMA_CSR_RX_DELAY_INT_CFG(base, idx)		(base+0x0210+RING_OFFSET(idx))

/**0x1000**/
#define QDMA_CSR_LMGR_INIT_CFG(base)				(base+0x1000)

#define INT_STATUS_RX_DONE							(1<<0)
#define INT_MASK_RX_DONE							(1<<0)

/* borrow QDMA WAN DBG_MEM_XS_DATA_HI for storing qdma init param*/
#define QDMA_WAN_DBG_MEM_XS_DATA_HI					(0x1fb57108)
//#define FLASH_QDMA_INIT								(0xffb4 | flash_base)


/* register operation*/
#define qdmaSetTxDscpBase(base, idx, val)	IO_SREG(QDMA_CSR_TX_DSCP_BASE(base, idx), val)
#define qdmaGetTxDscpBase(base, idx)		IO_GREG(QDMA_CSR_TX_DSCP_BASE(base, idx))
#define qdmaSetRxDscpBase(base, idx, val)	IO_SREG(QDMA_CSR_RX_DSCP_BASE(base, idx), val)
#define qdmaGetRxDscpBase(base, idx)		IO_GREG(QDMA_CSR_RX_DSCP_BASE(base, idx))
#define qdmaSetTxCpuIdx(base, idx, val)		IO_SMASK(QDMA_CSR_TX_CPU_IDX(base, idx), TX_CPU_IDX_MASK, TX_CPU_IDX_SHIFT, val)
#define qdmaGetTxCpuIdx(base, idx)			IO_GMASK(QDMA_CSR_TX_CPU_IDX(base, idx), TX_CPU_IDX_MASK, TX_CPU_IDX_SHIFT)
#define qdmaSetTxDmaIdx(base, idx, val)		IO_SMASK(QDMA_CSR_TX_DMA_IDX(base, idx), TX_DMA_IDX_MASK, TX_DMA_IDX_SHIFT, val)
#define qdmaGetTxDmaIdx(base, idx)			IO_GMASK(QDMA_CSR_TX_DMA_IDX(base, idx), TX_DMA_IDX_MASK, TX_DMA_IDX_SHIFT)
#define qdmaSetRxCpuIdx(base, idx, val)		IO_SMASK(QDMA_CSR_RX_CPU_IDX(base, idx), RX_CPU_IDX_MASK, RX_CPU_IDX_SHIFT, val)
#define qdmaGetRxCpuIdx(base, idx)			IO_GMASK(QDMA_CSR_RX_CPU_IDX(base, idx), RX_CPU_IDX_MASK, RX_CPU_IDX_SHIFT)
#define qdmaSetRxDmaIdx(base, idx, val)		IO_SMASK(QDMA_CSR_RX_DMA_IDX(base, idx), RX_DMA_IDX_MASK, RX_DMA_IDX_SHIFT, val)
#define qdmaGetRxDmaIdx(base, idx)			IO_GMASK(QDMA_CSR_RX_DMA_IDX(base, idx), RX_DMA_IDX_MASK, RX_DMA_IDX_SHIFT)

#define qdmaSetRxRingSize(base, idx, val)	IO_SMASK(QDMA_CSR_RX_RING_SIZE(base, idx), RX_RING_SIZE_MASK, RX_RING_SIZE_SHIFT, val)
#define qdmaGetRxRingSize(base, idx)		IO_GMASK(QDMA_CSR_RX_RING_SIZE(base, idx), RX_RING_SIZE_MASK, RX_RING_SIZE_SHIFT)
#define qdmaSetRxRingThrh(base, idx, val)	IO_SMASK(QDMA_CSR_RX_RING_SIZE(base,idx), RX_RING_LOW_THR_MASK, RX_RING_LOW_THR_SHIFT, val)
#define qdmaGetRxRingThrh(base, idx)		IO_GMASK(QDMA_CSR_RX_RING_SIZE(base,idx), RX_RING_LOW_THR_MASK, RX_RING_LOW_THR_SHIFT)

#define qdmaSetHwDscpBase(base, val)	IO_SREG(QDMA_CSR_HWFWD_DSCP_BASE(base), val)
#define qdmaGetHwDscpBase(base)			IO_GREG(QDMA_CSR_HWFWD_DSCP_BASE(base))
#define qdmaSetHwBuffBase(base, val)	IO_SREG(QDMA_CSR_HWFWD_BUFF_BASE(base), val)
#define qdmaGetHwBuffBase(base)			IO_GREG(QDMA_CSR_HWFWD_BUFF_BASE(base))
#define qdmaSetHwPayloadSize(base, val)	IO_SMASK(QDMA_CSR_HWFWD_DSCP_CFG(base), HWFWD_PAYLOAD_SIZE_MASK, HWFWD_PAYLOAD_SIZE_SHIFT, val)
#define qdmaGetHwPayloadSize(base)		IO_GMASK(QDMA_CSR_HWFWD_DSCP_CFG(base), HWFWD_PAYLOAD_SIZE_MASK, HWFWD_PAYLOAD_SIZE_SHIFT)
#define qdmaSetHwLowThrshld(base, val)	IO_SMASK(QDMA_CSR_HWFWD_DSCP_CFG(base), HWFWD_DSCP_LOW_THRSHLD_MASK, HWFWD_DSCP_LOW_THRSHLD_SHIFT, val)
#define qdmaGetHwLowThrshld(base)		IO_GMASK(QDMA_CSR_HWFWD_DSCP_CFG(base), HWFWD_DSCP_LOW_THRSHLD_MASK, HWFWD_DSCP_LOW_THRSHLD_SHIFT)
#define qdmaSetHwDscpNum(base, val)		IO_SMASK(QDMA_CSR_LMGR_INIT_CFG(base), HWFWD_DSCP_NUM_MASK, HWFWD_DSCP_NUM_SHIFT, val)
#define qdmaGetHwDscpNum(base)			IO_GMASK(QDMA_CSR_LMGR_INIT_CFG(base), HWFWD_DSCP_NUM_MASK, HWFWD_DSCP_NUM_SHIFT)

#define qdmaSetHwInitCfg(base, val)		IO_SREG(QDMA_CSR_LMGR_INIT_CFG(base), val)
#define qdmaGetHwInitCfg(base)			IO_GREG(QDMA_CSR_LMGR_INIT_CFG(base))

#define qdmaSetHWInitStart(base)		IO_SBITS(QDMA_CSR_LMGR_INIT_CFG(base), LMGR_INIT_START)
#define qdmaGetHWInitStart(base)		(IO_GREG(QDMA_CSR_LMGR_INIT_CFG(base)) & LMGR_INIT_START)
#define qdmaSetGlbCfg(base, val)		IO_SREG(QDMA_CSR_GLB_CFG(base), val)
#define qdmaGetGlbCfg(base)				IO_GREG(QDMA_CSR_GLB_CFG(base))
#define qdmaGetIrqEntryLen(base)		IO_GMASK(QDMA_CSR_IRQ_STATUS(base), IRQ_STATUS_ENTRY_LEN_MASK, IRQ_STATUS_ENTRY_LEN_SHIFT)
#define qdmaGetIrqStatus(base)			IO_GREG(QDMA_CSR_IRQ_STATUS(base))
#define qdmaSetIrqBase(base, val)		IO_SREG(QDMA_CSR_IRQ_BASE(base), val)
#define qdmaSetIrqDepth(base, val)		IO_SMASK(QDMA_CSR_IRQ_CFG(base), IRQ_CFG_DEPTH_MASK, IRQ_CFG_DEPTH_SHIFT, val)
#define qdmaSetIrqClearLen(base, val)	IO_SMASK(QDMA_CSR_IRQ_CLEAR_LEN(base), IRQ_CLEAR_LEN_MASK, IRQ_CLEAR_LEN_SHIFT, val)

#define qdmaEnableRxDma(base)			IO_SBITS(QDMA_CSR_GLB_CFG(base), GLB_CFG_RX_DMA_EN)
#define qdmaDisableRxDma(base)			IO_CBITS(QDMA_CSR_GLB_CFG(base), GLB_CFG_RX_DMA_EN)
#define qdmaEnableTxDma(base)			IO_SBITS(QDMA_CSR_GLB_CFG(base), GLB_CFG_TX_DMA_EN)
#define qdmaDisableTxDma(base)			IO_CBITS(QDMA_CSR_GLB_CFG(base), GLB_CFG_TX_DMA_EN)

#define qdmaClearIntStatus1(base, val)	IO_SREG(QDMA_CSR_INT_STATUS1(base), val)
#define qdmaGetIntStatus1(base)			IO_GREG(QDMA_CSR_INT_STATUS1(base))
#define qdmaClearIntStatus2(base, val)	IO_SREG(QDMA_CSR_INT_STATUS2(base), val)
#define qdmaGetIntStatus2(base)			IO_GREG(QDMA_CSR_INT_STATUS2(base))


typedef struct macMemPool_s
{
	u8 descrPool[MEM_POOL_SIZE];	/* Descr pool area */
} macMemPool_t;

/* ----- Ethernet Link Profile ----- */
typedef struct macPhyLinkProfile_s {
	u32 linkSpeed;							/* 10Mbps or 100Mbps */
	u32 duplexMode;							/* Half/Full Duplex Mode */
	u32 enetMode;
	u32 ANCompFlag;							/* auto_negotiation complete Flag */
	u32 PollCount;							/* auto_negotiation polling check count */
} macPhyLinkProfile_t;


typedef struct macAdapter_s
{
  u8  macAddr[6];  /* MAC-Address */
  macPhyLinkProfile_t *macPhyLinkProfile_p;
  macMemPool_t        *macMemPool_p;
  u32 enetPhyAddr;
  u8  enetPhyId;
} macAdapter_t;


#define uint u32
#define uint32 u32
#define ushort u16


#if defined(TCSUPPORT_CPU_EN7580)
typedef struct {
	uint	pkt_addr ;

	struct {
  #ifdef __BIG_ENDIAN
		uint ctx		: 1 ;
		uint ctx_ring		: 3 ;
		uint ctx_idx		: 12 ;
		uint pkt_len		: 16 ;
  #else
		uint pkt_len		: 16 ;
		uint ctx_idx		: 12 ;
		uint ctx_ring		: 3 ;
		uint ctx		: 1 ;
  #endif /* __BIG_ENDIAN */

  #ifdef __BIG_ENDIAN
		uint resv0		: 16 ;
		uint mul_first_idx	: 16 ;
  #else
		uint mul_first_idx	: 16 ;
		uint resv0		: 16 ;
  #endif /* __BIG_ENDIAN */

  #ifdef __BIG_ENDIAN
		uint resv1		: 29 ;
		uint mul_remain_pktnum	: 3 ;
  #else
		uint mul_remain_pktnum	: 3 ;
		uint resv1		: 29 ;
  #endif /* __BIG_ENDIAN */
	} ctrl ;

	uint	msg[2] ;

	uint	resv2 ;
	uint	resv3 ;
} QDMA_HWFWD_DMA_DSCP_T ;

#else
typedef struct {
	uint	pkt_addr ;
	struct {
  #ifdef __BIG_ENDIAN
		uint ctx		: 1 ;
		uint resv		: 2 ;
		uint ctx_ring		: 1 ;
		uint ctx_idx		: 12 ;
		uint pkt_len		: 16 ;
  #else
		uint pkt_len		: 16 ;
		uint ctx_idx		: 12 ;
		uint ctx_ring		: 1 ;
		uint resv		: 2 ;
		uint ctx		: 1 ;
  #endif /* __BIG_ENDIAN */
	} ctrl ;
	uint msg[2] ;
} QDMA_HWFWD_DMA_DSCP_T ;
#endif

typedef struct {
	ushort	IrqQueueAsynchronous ;
	ushort	txIrqQueueIdxErrs ;
	uint	txCounts ;
	uint	txRecycleCounts ;
	uint	txQdmaDropCounts ;
	uint	rxCounts ;
	ushort	txDscpIncorrect ;
	ushort	rxDscpIncorrect ;
	ushort	rxPktErrs ;
	ushort	noTxDscps ;
	ushort	noRxDscps ;
} BM_Counters_T ;

typedef union {
	struct {
#if defined(TCSUPPORT_CPU_EN7523)
  #ifdef __BIG_ENDIAN
		uint			: 1 ;
		uint mic_idx		: 1 ;
		uint sp_tag		: 16 ; /*sepcial-tag for LAN, gem_port for WAN*/
		uint ico		: 1;
		uint uco		: 1;
		uint tco		: 1;
		uint tso		: 1;
		uint fast		: 1 ;
		uint oam		: 1 ;
		uint channel		: 5 ;
		uint queue		: 3 ;
  #else
		uint queue		: 3 ;
		uint channel		: 5 ;
		uint oam		: 1 ;
		uint fast		: 1 ;
		uint tso		: 1;
		uint tco		: 1;
		uint uco		: 1;
		uint ico		: 1;
		uint sp_tag		: 16 ; /*sepcial-tag for LAN, gem_port for WAN*/
		uint mic_idx		: 1 ;
		uint			: 1 ;
  #endif
#elif defined(TCSUPPORT_CPU_EN7580)
  #ifdef __BIG_ENDIAN
		uint			: 1 ;
		uint mic_idx		: 1 ;
		uint sp_tag		: 16 ; /*sepcial-tag for LAN, gem_port for WAN*/
		uint			: 4 ;
		uint fast		: 1 ;
		uint oam		: 1 ;
		uint channel		: 5 ;
		uint queue		: 3 ;
  #else
		uint queue		: 3 ;
		uint channel		: 5 ;
		uint oam		: 1 ;
		uint fast		: 1 ;
		uint			: 4 ;
		uint sp_tag		: 16 ; /*sepcial-tag for LAN, gem_port for WAN*/
		uint mic_idx		: 1 ;
		uint			: 1 ;
  #endif
#else
  #ifdef __BIG_ENDIAN
		uint32 resv1		: 1;
		uint32 tsid		: 5;
		uint32 tse		: 1;
		uint32 dei		: 1;
		uint32 resv2		: 12;
		uint32 oam		: 1;
		uint32 channel		: 8;
		uint32 queue		: 3;
  #else
		uint32 queue		: 3;
		uint32 channel		: 8;
		uint32 oam		: 1;
		uint32 resv2		: 12;
		uint32 dei		: 1;
		uint32 tse		: 1;
		uint32 tsid		: 5;
		uint32 resv1		: 1;
  #endif /* __BIG_ENDIAN */
#endif

#if defined(TCSUPPORT_CPU_EN7523)
  #ifdef __BIG_ENDIAN
		uint32 no_drop		: 1; /*means not be dropped by QDMA*/
		uint32 mtr_g		: 7; /*0x7f means not use meter*/
		uint32 fport		: 4;
		uint32 nboq		: 5;
		uint32			: 5;
		uint32 act_g		: 10;
  #else
		uint32 act_g		: 10;
		uint32			: 5;
		uint32 nboq		: 5;
		uint32 fport		: 4;
		uint32 mtr_g		: 7; /*0x7f means not use meter*/
		uint32 no_drop		: 1; /*means not be dropped by QDMA*/
  #endif /* __BIG_ENDIAN */
#elif defined(TCSUPPORT_CPU_EN7580)
  #ifdef __BIG_ENDIAN
		uint32 no_drop		: 1; /*means not be dropped by QDMA*/
		uint32 mtr_g		: 7; /*0x7f means not use meter*/
		uint32 fport		: 3;
		uint32 nboq		: 5;
		uint32			: 6;
		uint32 udf_pmap		: 6;
		uint32 ico		: 1;
		uint32 uco		: 1;
		uint32 tco		: 1;
		uint32 tso		: 1;
  #else
		uint32 tso		: 1;
		uint32 tco		: 1;
		uint32 uco		: 1;
		uint32 ico		: 1;
		uint32 udf_pmap		: 6;
		uint32			: 6;
		uint32 nboq		: 5;
		uint32 fport		: 3;
		uint32 mtr_g		: 7; /*0x7f means not use meter*/
		uint32 no_drop		: 1; /*means not be dropped by QDMA*/
  #endif /* __BIG_ENDIAN */
#else
  #ifdef __BIG_ENDIAN
		uint32 ico		: 1;
		uint32 uco		: 1;
		uint32 tco		: 1;
		uint32 tso		: 1;
		uint32 resv3		: 6;
		uint32 fport		: 3;
		uint32 vlanEn		: 1;
		uint32 vlanTpID		: 2;
		uint32 vlanTag		: 16;
  #else
		uint32 vlanTag		: 16;
		uint32 vlanTpID		: 2;
		uint32 vlanEn		: 1;
		uint32 fport		: 3;
		uint32 resv3		: 6;
		uint32 tso		: 1;
		uint32 tco		: 1;
		uint32 uco		: 1;
		uint32 ico		: 1;
  #endif /* __BIG_ENDIAN */
#endif
	} raw ;
	uint msg[2] ;
} ethTxMsg_t ;

typedef union
{
	uint32 word;

} rxMsgWord0_t;

typedef union
{
	uint32 word;

} rxMsgWord1_t;

typedef union
{
	uint32 word;

} rxMsgWord2_t;


typedef struct ethRxMsg_s
{
	rxMsgWord0_t	rxMsgW0;
	rxMsgWord1_t	rxMsgW1;
	rxMsgWord2_t	rxMsgW2;
	uint32		resv;
} ethRxMsg_t;


typedef struct {
	uint resv1 ;
	struct {
#ifdef __BIG_ENDIAN
		uint done		: 1 ;
		uint drop_pkt		: 1 ;
  #if defined(TCSUPPORT_CPU_EN7580) || defined(TCSUPPORT_CPU_EN7527) || defined(TCSUPPORT_CPU_EN7516)
		uint nls		: 1 ;
		uint resv2		: 13 ;
  #else
		uint resv2		: 14 ;
  #endif
		uint pkt_len		: 16 ;
#else
		uint pkt_len		: 16 ;
  #if defined(TCSUPPORT_CPU_EN7580) || defined(TCSUPPORT_CPU_EN7527) || defined(TCSUPPORT_CPU_EN7516)
		uint resv2		: 13 ;
		uint nls		: 1 ;
  #else
		uint resv2		: 14 ;
  #endif
		uint drop_pkt		: 1 ;
		uint done		: 1 ;
#endif /* __BIG_ENDIAN */
	} ctrl ;
	uint pkt_addr ;
#ifdef __BIG_ENDIAN
	uint resv3			: 20 ;
	uint next_idx			: 12 ;
#else
	uint next_idx			: 12 ;
	uint resv3			: 20 ;
#endif /* __BIG_ENDIAN */
	uint msg[4] ;
} QDMA_DMA_DSCP_T ;


struct QDMA_DscpInfo_S {
	QDMA_DMA_DSCP_T			*dscpPtr ;
	uint				dscpIdx ;

	struct sk_buff				*skb ;
	struct QDMA_DscpInfo_S		*next ;
} ;

typedef struct {
	u32 						dbgLevel ;
	void *						csrBaseAddr ;
	u16						txDscpNum ;				/* Total TX DSCP number */
	u16						rxDscpNum ;				/* Total RX DSCP number */
	u16						hwFwdDscpNum ;
	u16						irqDepth ;				/* Max depth for IRQ queue */
	u16						hwPktSize ;
	void *				dscpInfoAddr ; 			/* Start pointer for DSCP information node */
	void *				txBaseAddr ;
	void *				rxBaseAddr ;
	void *				irqQueueAddr ;			/* IRQ queue address */
	void *				hwFwdBaseAddr ;			/* Base address of the hardware forwarding */
	void *				hwFwdBuffAddr ;			/* Base address of the hardware forwarding Buffer*/
	u32						hwFwdPayloadSize ;		/* Payload size of the hardware forwarding Buffer*/
	struct QDMA_DscpInfo_S		*txHeadPtr ;			/* Head node for unused tx desc. */
	struct QDMA_DscpInfo_S		*txTailPtr ;			/* Tail node for unused tx desc. */
	struct QDMA_DscpInfo_S		*txUsingPtr ;			/* TXDMA using DSCP node. */
	struct QDMA_DscpInfo_S		*rxStartPtr ;			/* Start using node for rx desc. */
	struct QDMA_DscpInfo_S		*rxEndPtr ;				/* End using node for rx desc. */
	struct QDMA_DscpInfo_S		*rxUsingPtr ;			/* RXDMA using DSCP node. */

	BM_Counters_T				counters ;

} QDMA_Private_T ;


struct airoha_reg_map {
	u32	tx_irq_mask;
	u32	tx_irq_status;
	struct {
		u32	rx_ptr;		/* rx base pointer */
		u32	rx_cnt_cfg;	/* rx max count configuration */
		u32	pcrx_ptr;	/* rx cpu pointer */
		u32	glo_cfg;	/* global configuration */
		u32	rst_idx;	/* reset index */
		u32	delay_irq;	/* delay interrupt */
		u32	irq_status;	/* interrupt status */
		u32	irq_mask;	/* interrupt mask */
		u32	adma_rx_dbg0;
		u32	int_grp;
	} pdma;
	struct {
		u32	qtx_cfg;	/* tx queue configuration */
		u32	qtx_sch;	/* tx queue scheduler configuration */
		u32	rx_ptr;		/* rx base pointer */
		u32	rx_cnt_cfg;	/* rx max count configuration */
		u32	qcrx_ptr;	/* rx cpu pointer */
		u32	glo_cfg;	/* global configuration */
		u32	rst_idx;	/* reset index */
		u32	delay_irq;	/* delay interrupt */
		u32	fc_th;		/* flow control */
		u32	int_grp;
		u32	hred;		/* interrupt mask */
		u32	ctx_ptr;	/* tx acquire cpu pointer */
		u32	dtx_ptr;	/* tx acquire dma pointer */
		u32	crx_ptr;	/* tx release cpu pointer */
		u32	drx_ptr;	/* tx release dma pointer */
		u32	fq_head;	/* fq head pointer */
		u32	fq_tail;	/* fq tail pointer */
		u32	fq_count;	/* fq free page count */
		u32	fq_blen;	/* fq free page buffer length */
		u32	tx_sch_rate;	/* tx scheduler rate control registers */
	} qdma;
	u32	gdm1_cnt;
	u32	gdma_to_ppe0;
	u32	ppe_base;
	u32	wdma_base[3];
	u32	pse_iq_sta;
	u32	pse_oq_sta;
};

/* struct mtk_eth_data -	This is the structure holding all differences
 *				among various plaforms
 * @reg_map			Soc register map.
 * @ana_rgc3:                   The offset for register ANA_RGC3 related to
 *				sgmiisys syscon
 * @caps			Flags shown the extra capability for the SoC
 * @hw_features			Flags shown HW features
 * @required_clks		Flags shown the bitmap for required clocks on
 *				the target SoC
 * @required_pctl		A bool value to show whether the SoC requires
 *				the extra setup for those pins used by GMAC.
 * @hash_offset			Flow table hash offset.
 * @version			SoC version.
 * @foe_entry_size		Foe table entry size.
 * @has_accounting		Bool indicating support for accounting of
 *				offloaded flows.
 * @txd_size			Tx DMA descriptor size.
 * @rxd_size			Rx DMA descriptor size.
 * @rx_irq_done_mask		Rx irq done register mask.
 * @rx_dma_l4_valid		Rx DMA valid register mask.
 * @dma_max_len			Max DMA tx/rx buffer length.
 * @dma_len_offset		Tx/Rx DMA length field offset.
 */
struct airoha_soc_data {
	const struct airoha_reg_map *reg_map;
	u32             ana_rgc3;
	u64		caps;
	u64		required_clks;
	bool		required_pctl;
	u8		offload_version;
	u8		hash_offset;
	u8		version;
	u16		foe_entry_size;
	netdev_features_t hw_features;
	bool		has_accounting;
	bool		disable_pll_modes;
	struct {
		u32	txd_size;
		u32	rxd_size;
		u32	rx_irq_done_mask;
		u32	rx_dma_l4_valid;
		u32	dma_max_len;
		u32	dma_len_offset;
	} txrx;
};

/* struct mtk_eth -	This is the main datasructure for holding the state
 *			of the driver
 * @dev:		The device pointer
 * @dev:		The device pointer used for dma mapping/alloc
 * @base:		The mapped register i/o base
 * @page_lock:		Make sure that register operations are atomic
 * @tx_irq__lock:	Make sure that IRQ register operations are atomic
 * @rx_irq__lock:	Make sure that IRQ register operations are atomic
 * @dim_lock:		Make sure that Net DIM operations are atomic
 * @dummy_dev:		we run 2 netdevs on 1 physical DMA ring and need a
 *			dummy for NAPI to work
 * @netdev:		The netdev instances
 * @mac:		Each netdev is linked to a physical MAC
 * @irq:		The IRQ that we are using
 * @msg_enable:		Ethtool msg level
 * @ethsys:		The register map pointing at the range used to setup
 *			MII modes
 * @infra:              The register map pointing at the range used to setup
 *                      SGMII and GePHY path
 * @sgmii_pcs:		Pointers to mtk-pcs-lynxi phylink_pcs instances
 * @sgmii_wrapped_pcs:	Pointers to NETSYSv3 wrapper PCS instances
 * @usxgmii_pll:	The register map pointing at the range used to control
 *			the USXGMII SerDes PLL
 * @regmap_pextp:	The register map pointing at the range used to setup
 *			PHYA
 * @usxgmii_pcs:	Pointer to array of pointers to struct for USXGMII PCS
 * @pctl:		The register map pointing at the range used to setup
 *			GMAC port drive/slew values
 * @dma_refcnt:		track how many netdevs are using the DMA engine
 * @tx_ring:		Pointer to the memory holding info about the TX ring
 * @rx_ring:		Pointer to the memory holding info about the RX ring
 * @rx_ring_qdma:	Pointer to the memory holding info about the QDMA RX ring
 * @tx_napi:		The TX NAPI struct
 * @rx_napi:		The RX NAPI struct
 * @rx_events:		Net DIM RX event counter
 * @rx_packets:		Net DIM RX packet counter
 * @rx_bytes:		Net DIM RX byte counter
 * @rx_dim:		Net DIM RX context
 * @tx_events:		Net DIM TX event counter
 * @tx_packets:		Net DIM TX packet counter
 * @tx_bytes:		Net DIM TX byte counter
 * @tx_dim:		Net DIM TX context
 * @scratch_ring:	Newer SoCs need memory for a second HW managed TX ring
 * @phy_scratch_ring:	physical address of scratch_ring
 * @scratch_head:	The scratch memory that scratch_ring points to.
 * @clks:		clock array for all clocks required
 * @mii_bus:		If there is a bus we need to create an instance for it
 * @pending_work:	The workqueue used to reset the dma ring
 * @state:		Initialization and runtime state of the device
 * @soc:		Holding specific data among vaious SoCs
 */

struct airoha_eth {
	struct device			*dev;
	struct device			*dma_dev;
	void __iomem			*base;
	void __iomem			*qdma_base[2];
	void __iomem			*rst_2;
	void __iomem			*esw;
	void				*sram_base;
	spinlock_t			page_lock;
	spinlock_t			tx_irq_lock;
	spinlock_t			rx_irq_lock;
	struct net_device		dummy_dev;
	struct net_device		*netdev[AIROHA_MAX_DEVS];
	struct airoha_mac			*mac[AIROHA_MAX_DEVS];
	int				irq[10];
	u32				msg_enable;
//	unsigned long			sysclk;
//	struct regmap			*ethsys;
//	struct regmap			*infra;
//	struct phylink_pcs		*sgmii_pcs[MTK_MAX_DEVS];
//	struct regmap			*toprgu;
//	struct regmap			*usxgmii_pll;
//	struct regmap			*regmap_pextp[AIROHA_MAX_DEVS];
//	struct mtk_usxgmii_pcs		*usxgmii_pcs[MTK_MAX_DEVS];
	struct regmap			*pctl;
	bool				hwlro;
	refcount_t			dma_refcnt;
//	struct mtk_tx_ring		tx_ring;
//	struct mtk_rx_ring		rx_ring[MTK_MAX_RX_RING_NUM];
//	struct mtk_rx_ring		rx_ring_qdma;
	struct napi_struct		tx_napi;
	struct napi_struct		rx_napi;
	void				*scratch_ring;
	dma_addr_t			phy_scratch_ring;
	void				*scratch_head;
//	struct clk			*clks[MTK_CLK_MAX];

	struct mii_bus			*mii_bus;
	struct work_struct		pending_work;
	unsigned long			state;

	const struct airoha_soc_data	*soc;

	spinlock_t			dim_lock;

	u32				rx_events;
	u32				rx_packets;
	u32				rx_bytes;
	struct dim			rx_dim;

	u32				tx_events;
	u32				tx_packets;
	u32				tx_bytes;
	struct dim			tx_dim;

	int				ip_align;

//	struct metadata_dst		*dsa_meta[MTK_MAX_DSA_PORTS];

//	struct mtk_ppe			*ppe[2];
	struct rhashtable		flow_table;

	struct bpf_prog			__rcu *prog;

// 	struct {
// 		struct delayed_work monitor_work;
// 		u32 wdidx;
// 		u8 wdma_hang_count;
// 		u8 qdma_hang_count;
// 		u8 adma_hang_count;
// 	} reset;
};

/* struct mtk_mac -	the structure that holds the info about the MACs of the
 *			SoC
 * @id:			The number of the MAC
 * @interface:		Interface mode kept for detecting change in hw settings
 * @of_node:		Our devicetree node
 * @hw:			Backpointer to our main datastruture
 * @hw_stats:		Packet statistics counter
 */
struct airoha_mac {
	int				id;
	phy_interface_t			interface;
	int				speed;
	struct device_node		*of_node;
	struct phylink			*phylink;
	struct phylink_config		phylink_config;
	struct airoha_eth			*hw;
//	struct mtk_hw_stats		*hw_stats;
//	__be32				hwlro_ip[MTK_MAX_LRO_IP_CNT];
	int				hwlro_ip_cnt;
	unsigned int			syscfg0;
	struct notifier_block		device_notifier;
};

struct airoha_qdma {
		u32	qtx_cfg;	/* tx queue configuration */
		u32	qtx_sch;	/* tx queue scheduler configuration */
		u32	rx_ptr;		/* rx base pointer */
		u32	rx_cnt_cfg;	/* rx max count configuration */
		u32	qcrx_ptr;	/* rx cpu pointer */
		u32	glo_cfg;	/* global configuration */
		u32	rst_idx;	/* reset index */
		u32	delay_irq;	/* delay interrupt */
		u32	fc_th;		/* flow control */
		u32	int_grp;
		u32	hred;		/* interrupt mask */
		u32	ctx_ptr;	/* tx acquire cpu pointer */
		u32	dtx_ptr;	/* tx acquire dma pointer */
		u32	crx_ptr;	/* tx release cpu pointer */
		u32	drx_ptr;	/* tx release dma pointer */
		u32	fq_head;	/* fq head pointer */
		u32	fq_tail;	/* fq tail pointer */
		u32	fq_count;	/* fq free page count */
		u32	fq_blen;	/* fq free page buffer length */
		u32	tx_sch_rate;	/* tx scheduler rate control registers */
};

enum airoha_dev_state {
	AIROHA_HW_INIT,
	AIROHA_RESETTING
};

/* PSE Port Definition */
enum airoha_pse_port {
	PSE_ADMA_PORT = 0,
	PSE_GDM1_PORT,
	PSE_GDM2_PORT,
	PSE_PPE0_PORT,
	PSE_PPE1_PORT,
	PSE_QDMA_TX_PORT,
	PSE_QDMA_RX_PORT,
	PSE_DROP_PORT,
	PSE_WDMA0_PORT,
	PSE_WDMA1_PORT,
	PSE_TDMA_PORT,
	PSE_NONE_PORT,
	PSE_PPE2_PORT,
	PSE_WDMA2_PORT,
	PSE_EIP197_PORT,
	PSE_GDM3_PORT,
	PSE_PORT_MAX
};

/* define GDMA port */
#define GDM_P_PDMA		(0x0)
#define GDM_P_GDMA1		(0x1)
#define GDM_P_GDMA2		(0x2)
#define GDM_P_GDMA3		(0x3)
#define GDM_P_GDMA4		(0x9)
#define GDM_P_PPE		(0x4)
#define GDM_P_QDMA		(0x5)
#define GDM_P_DISCARD		(0x7)
#define GDM_P_CPU		GDM_P_PDMA

/* GMAC Identifier */
enum airoha_gmac_id {
	AIROHA_GMAC1_ID = 0,
	AIROHA_GMAC2_ID,
	AIROHA_GMAC3_ID,
	AIROHA_GMAC_ID_MAX
};



/* QDMA_CSR_HWFWD_DSCP_CFG(base) */
#define HWFWD_PAYLOAD_SIZE_SHIFT					(28)
#define HWFWD_PAYLOAD_SIZE_MASK						(0x3<<HWFWD_PAYLOAD_SIZE_SHIFT)
#define HWFWD_PAYLOAD_SIZE_2K						(0x0)
#define HWFWD_PAYLOAD_SIZE_4K						(0x1)
#define HWFWD_PAYLOAD_SIZE_8K						(0x2)
#define HWFWD_PAYLOAD_SIZE_16K						(0x3)
#define HWFWD_DSCP_LOW_THRSHLD_SHIFT				(0)
#define HWFWD_DSCP_LOW_THRSHLD_MASK					(0x1FFF<<HWFWD_DSCP_LOW_THRSHLD_SHIFT)

/* QDMA_CSR_INT_STATUS(base) */
#define INT_STATUS_XPON_PHY							(1<<24)
#define INT_STATUS_EPON_MAC							(1<<17)
#define INT_STATUS_GPON_MAC							(1<<16)
#define INT_STATUS_RX1_COHERENT						(1<<15)
#define INT_STATUS_TX1_COHERENT						(1<<14)
#define INT_STATUS_RX0_COHERENT						(1<<13)
#define INT_STATUS_TX0_COHERENT						(1<<12)
#define INT_STATUS_HWFWD_DSCP_LOW					(1<<10)
#define INT_STATUS_IRQ_FULL							(1<<9)
#define INT_STATUS_HWFWD_DSCP_EMPTY					(1<<8)
#define INT_STATUS_NO_RX1_CPU_DSCP					(1<<7)
#define INT_STATUS_NO_TX1_CPU_DSCP					(1<<6)
#define INT_STATUS_RX1_DONE							(1<<5)
#define INT_STATUS_TX1_DONE							(1<<4)
#define INT_STATUS_NO_RX0_CPU_DSCP					(1<<3)
#define INT_STATUS_NO_TX0_CPU_DSCP					(1<<2)
#define INT_STATUS_RX0_DONE							(1<<1)
#define INT_STATUS_TX0_DONE							(1<<0)
#define INT_STATUS_QDMA_DONE						(0x00000033)
#define INT_STATUS_QDMA_FAULT						(0x0000F740)

/* QDMA_CSR_GLB_CFG(base) */
#define GLB_CFG_RX_2B_OFFSET						(1<<31)
#define GLB_CFG_DMA_PREFERENCE_SHIFT				(29)
#define GLB_CFG_DMA_PREFERENCE_MASK					(0x3<<GLB_CFG_DMA_PREFERENCE_SHIFT)
#define PREFER_ROIND_ROBIN							(0x00)
#define PREFER_FWD_TX1_TX0							(0x01)
#define PREFER_TX1_FWD_TX0							(0x10)
#define PREFER_TX1_TX0_FWD							(0x11)
#define GLB_CFG_MSG_WORD_SWAP						(1<<28)
#define GLB_CFG_DSCP_BYTE_SWAP						(1<<27)
#define GLB_CFG_PAYLOAD_BYTE_SWAP					(1<<26)
#define GLB_CFG_SLM_RELEASE_EN						(1<<21)
#define GLB_CFG_TX_IMMEDIATE_DONE					(1<<20)
#define GLB_CFG_IRQ_EN								(1<<19)
#define GLB_CFG_LOOPCNT_EN							(1<<18)
#define GLB_CFG_UMAC_LOOPBACK						(1<<17)
#define GLB_CFG_QDMA_LOOPBACK						(1<<16)
#define GLB_CFG_CHECK_DONE							(1<<7)
#define GLB_CFG_TX_WB_DONE							(1<<6)
#define GLB_CFG_BST_SE_SHIFT						(4)
#define GLB_CFG_BST_SE_MASK							(0x3<<GLB_CFG_BST_SE_SHIFT)
#define VAL_BST_4_DWORD								(0x0)
#define VAL_BST_8_DWORD								(0x1)
#define VAL_BST_16_DWARD							(0x2)
#define VAL_BST_32_DWARD							(0x3)
#define GLB_CFG_RX_DMA_BUSY							(1<<3)
#define GLB_CFG_RX_DMA_EN							(1<<2)
#define GLB_CFG_TX_DMA_BUSY							(1<<1)
#define GLB_CFG_TX_DMA_EN							(1<<0)

/* QDMA_CSR_TX_CPU_IDX(base) */
#define TX_CPU_IDX_SHIFT							(0)
#define TX_CPU_IDX_MASK								(0xFFF<<TX_CPU_IDX_SHIFT)

/* QDMA_CSR_TX_DMA_IDX(base) */
#define TX_DMA_IDX_SHIFT							(0)
#define TX_DMA_IDX_MASK								(0xFFF<<TX_DMA_IDX_SHIFT)

/* QDMA_CSR_RX_CPU_IDX(base) */
#define RX_CPU_IDX_SHIFT							(0)
#define RX_CPU_IDX_MASK								(0xFFF<<RX_CPU_IDX_SHIFT)

/* QDMA_CSR_RX_DMA_IDX(base) */
#define RX_DMA_IDX_SHIFT							(0)
#define RX_DMA_IDX_MASK								(0xFFF<<RX_DMA_IDX_SHIFT)

/* QDMA_CSR_LMGR_INIT_CFG(base) */
#define LMGR_INIT_START								(1<<31)
#define LMGR_ROM_MODE									(1<<30)	/*0:DRAM 64K+16K; 1:SRAM 16K+4K*/

#define HWFWD_DSCP_NUM_SHIFT						(0)
#define HWFWD_DSCP_NUM_MASK							(0x1FFF<<HWFWD_DSCP_NUM_SHIFT)

/* QDMA_CSR_RX_RING_CFG & QDMA_CSR_RX_RING_THR */
#define RX_RING_SIZE_SHIFT							(0)
#define RX_RING_SIZE_MASK							(0xFFF<<RX_RING_SIZE_SHIFT)
#define RX_RING_LOW_THR_SHIFT						(16)
#define RX_RING_LOW_THR_MASK						(0xFFF<<RX_RING_LOW_THR_SHIFT)


/* QDMA_CSR_IRQ_CFG(base) */
#define IRQ_CFG_THRESHOLD_SHIFT						(16)
#define IRQ_CFG_THRESHOLD_MASK						(0xFFF<<IRQ_CFG_THRESHOLD_SHIFT)
#define IRQ_CFG_DEPTH_SHIFT							(0)
#define IRQ_CFG_DEPTH_MASK							(0xFFF<<IRQ_CFG_DEPTH_SHIFT)

/* QDMA_CSR_IRQ_CLEAR_LEN(base) */
#define IRQ_CLEAR_LEN_SHIFT							(0)
#define IRQ_CLEAR_LEN_MASK							(0xFF<<IRQ_CLEAR_LEN_SHIFT)

/* QDMA_CSR_IRQ_STATUS(base) */
#define IRQ_STATUS_ENTRY_LEN_SHIFT					(16)
#define IRQ_STATUS_ENTRY_LEN_MASK					(0xFFF<<IRQ_STATUS_ENTRY_LEN_SHIFT)
#define IRQ_STATUS_HEAD_IDX_SHIFT					(0)
#define IRQ_STATUS_HEAD_IDX_MASK					(0xFFF<<IRQ_STATUS_HEAD_IDX_SHIFT)

/*******************************************************************
 ***                                       QDMA Reg Realted End                                         ***
 *******************************************************************/

/* GDMA1_FWD_CFG or GDMA2_FWD_CFG */
#define GDM_JMB_LEN_SHIFT				(28)
#define GDM_JMB_LEN						(0xf<<GDM_JMB_LEN_SHIFT)
#define GDM_20US_TICK_SLT				(1<<25)

#define GDM_INSV_EN						(1<<26)
#define GDM_UNTAG_EN					(1<<25)
#define GDM_STAG_EN						(1<<24)
#define GDM_ICS_EN						(1<<22)
#define GDM_TCS_EN						(1<<21)
#define GDM_UCS_EN						(1<<20)
#define GDM_DROP_256B					(1<<19)
#define GDM_DISPAD						(1<<18)
#define GDM_DISCRC						(1<<17)
#define GDM_STRPCRC						(1<<16)
#define GDM_UFRC_P_SHIFT				(12)
#define GDM_UFRC_P						(0xf<<GDM_UFRC_P_SHIFT)
#define GDM_BFRC_P_SHIFT				(8)
#define GDM_BFRC_P						(0xf<<GDM_BFRC_P_SHIFT)
#define GDM_MFRC_P_SHIFT				(4)
#define GDM_MFRC_P						(0xf<<GDM_MFRC_P_SHIFT)
#define GDM_OFRC_P_SHIFT				(0)
#define GDM_OFRC_P						(0xf<<GDM_MFRC_P_SHIFT)




//switch define
/* GSW_MFC */
#define MFC_BC_FFP_SHIFT	(24)
#define MFC_BC_FFP		(0xff<<MFC_BC_FFP_SHIFT)
#define MFC_UNM_FFP_SHIFT	(16)
#define MFC_UNM_FFP		(0xff<<MFC_UNM_FFP_SHIFT)
#define MFC_UNU_FFP_SHIFT	(8)
#define MFC_UNU_FFP		(0xff<<MFC_UNU_FFP_SHIFT)
#define MFC_CPU_EN		(1<<7)
#define MFC_CPU_PORT_SHIFT	(4)
#define MFC_CPU_PORT		(0x7<<MFC_CPU_PORT_SHIFT)
#define MFC_MIRROR_EN		(1<<3)
#define MFC_MIRROR_PORT_SHIFT	(0)
#define MFC_MIRROT_PORT		(0x7<<MFC_MIRROR_PORT_SHIFT)

/* GSW_PMCR */
#define IPG_CFG_PN_SHIFT	(18)
#define IPG_CFG_PN		(0x3<<IPG_CFG_PN_SHIFT)
#define EXT_PHY_PN		(1<<17)
#define MAC_MODE_PN		(1<<16)
#define FORCE_MODE_PN		(1<<15)
#define MAC_TX_EN_PN		(1<<14)
#define MAC_RX_EN_PN		(1<<13)
#define RGMII_MODE_PN		(1<<12)
#define BKOFF_EN_PN		(1<<9)
#define BACKPR_EN_PN		(1<<8)
#define ENABLE_EEE1G_PN		(1<<7)
#define ENABLE_EEE100_PN	(1<<6)
#define ENABLE_RX_FC_PN		(1<<5)
#define ENABLE_TX_FC_PN		(1<<4)
#define FORCE_SPD_PN_SHIFT	(2)
#define FORCE_SPD_PN		(0x3<<FORCE_SPD_PN_SHIFT)
#define FORCE_DPX_PN		(1<<1)
#define FORCE_LNK_PN		(1<<0)

#define IPG_CFG_NORMAL		(0)
#define IPG_CFG_SHORT		(1)
#define IPG_CFG_64BITS		(0x2)

#define PN_SPEED_10M		(0)
#define PN_SPEED_100M		(1)
#define PN_SPEED_1000M		(2)

/* GSW_PMSR */
#define EEE1G_STS		(1<<7)
#define EEE100_STS		(1<<6)
#define RX_FC_STS		(1<<5)
#define TX_FC_STS		(1<<4)
#define MAC_SPD_STS_SHIFT	(2)
#define MAC_SPD_STS		(0x3<<MAC_SPD_STS_SHIFT)
#define MAC_DPX_STS		(1<<1)
#define MAC_LINK_STS		(1<<0)









#endif /* AIROHA_ETH_H */
