// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 *   Copyright (C) 2024 Benjamin Larsson <benjamin.larsson@genexis.eu>
 *
 *   Heavily based on mtk_eth_soc.c
 */

#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/if_vlan.h>
#include <linux/reset.h>
#include <linux/tcp.h>
#include <linux/interrupt.h>
#include <linux/pinctrl/devinfo.h>
#include <linux/phylink.h>
#include <linux/jhash.h>
#include <linux/bitfield.h>
#include <net/dsa.h>
#include <net/dst_metadata.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
// #include <linux/version.h>
#include <asm/cacheflush.h>


extern void v7_flush_kern_cache_all(void);

#include "airoha_eth_soc.h"

static int airoha_msg_level = -1;
module_param_named(msg_level, airoha_msg_level, int, 0);
MODULE_PARM_DESC(msg_level, "Message level (-1=defaults,0=none,...,16=all)");

#define AIROHA_ETHTOOL_STAT(x) { #x, \
			      offsetof(struct mtk_hw_stats, x) / sizeof(u64) }

#define AIROHA_ETHTOOL_XDP_STAT(x) { #x, \
				  offsetof(struct mtk_hw_stats, xdp_stats.x) / \
				  sizeof(u64) }


static QDMA_Private_T DummygpQdmaPriv;
QDMA_Private_T *gpQdmaPriv = &DummygpQdmaPriv;
static macAdapter_t DummyAdapter;
macAdapter_t *mac_p = &DummyAdapter;
#define CONFIG_SYS_CACHELINE_SIZE	64 ((__aligned__(CONFIG_SYS_CACHELINE_SIZE)));
#define CONFIG_RX_2B_OFFSET 1

static const struct airoha_reg_map en7523_reg_map = {
	.tx_irq_mask		= 0x461c,
	.tx_irq_status		= 0x4618,
	.qdma = {
		.qtx_cfg	= 0x4400,
		.qtx_sch	= 0x4404,
		.rx_ptr		= 0x4500,
		.rx_cnt_cfg	= 0x4504,
		.qcrx_ptr	= 0x4508,
		.glo_cfg	= 0x4604,
		.rst_idx	= 0x4608,
		.delay_irq	= 0x460c,
		.fc_th		= 0x4610,
		.int_grp	= 0x4620,
		.hred		= 0x4644,
		.ctx_ptr	= 0x4700,
		.dtx_ptr	= 0x4704,
		.crx_ptr	= 0x4710,
		.drx_ptr	= 0x4714,
		.fq_head	= 0x4720,
		.fq_tail	= 0x4724,
		.fq_count	= 0x4728,
		.fq_blen	= 0x472c,
		.tx_sch_rate	= 0x4798,
	},
	.gdm1_cnt		= 0x1c00,
	.gdma_to_ppe0		= 0x3333,
	.ppe_base		= 0x2000,
	.wdma_base = {
		[0]		= 0x4800,
		[1]		= 0x4c00,
	},
	.pse_iq_sta		= 0x0180,
	.pse_oq_sta		= 0x01a0,
};

static const struct airoha_soc_data en7523_data = {
	.reg_map = &en7523_reg_map,
//	.caps = EN7523_CAPS,
//	.hw_features = MTK_HW_FEATURES,
//	.required_clks = MT7621_CLKS_BITMAP,
//	.required_pctl = false,
//	.version = 1,
//	.offload_version = 1,
//	.hash_offset = 2,
//	.foe_entry_size = MTK_FOE_ENTRY_V1_SIZE,
// 	.txrx = {
// 		.txd_size = sizeof(struct mtk_tx_dma),
// 		.rxd_size = sizeof(struct mtk_rx_dma),
// 		.rx_irq_done_mask = MTK_RX_DONE_INT,
// 		.rx_dma_l4_valid = RX_DMA_L4_VALID,
// 		.dma_max_len = MTK_TX_DMA_BUF_LEN,
// 		.dma_len_offset = 16,
// 	},
};

static bool airoha_uses_dsa(struct net_device *dev)
{
#if IS_ENABLED(CONFIG_NET_DSA)
	return netdev_uses_dsa(dev) &&
	       dev->dsa_ptr->tag_ops->proto == DSA_TAG_PROTO_MTK;
#else
	return false;
#endif
}

int qdma_bm_dump_dscp( void ){

	int idx = 0; 
	struct QDMA_DscpInfo_S *diPtr=NULL ;
	idx = 1 ;
	diPtr = gpQdmaPriv->rxStartPtr ;
	printk("\nRx%d DSCP Ring: RxStartIdx:%d, RxEndIdx:%d\n", 0, gpQdmaPriv->rxStartPtr->dscpIdx, gpQdmaPriv->rxEndPtr->dscpIdx) ;
	do {
		if(diPtr) {
			printk("%d: DSCP Idx:%d, DSCP Ptr:%.8x, Done Bit:%d, PktLen:%d, PktAddr:%.8x(%.8x), Next Idx:%d\n", 
																idx, diPtr->dscpIdx, 
																(uint)diPtr->dscpPtr, 
																diPtr->dscpPtr->ctrl.done,
																diPtr->dscpPtr->ctrl.pkt_len, 
																(uint)diPtr->skb, 
																(uint)diPtr->dscpPtr->pkt_addr,
																diPtr->dscpPtr->next_idx) ;
			diPtr = diPtr->next ;
			idx++ ;
		}
	} while(diPtr!=NULL && diPtr!=gpQdmaPriv->rxStartPtr) ;

	idx = 1 ;
	diPtr = gpQdmaPriv->txHeadPtr ;
	printk("\nTx%d DSCP Ring: TxHeadIdx:%d, txTailIdx:%d\n", 0, gpQdmaPriv->txHeadPtr->dscpIdx, gpQdmaPriv->txTailPtr->dscpIdx) ;
	do {
		if(diPtr) {
			printk("%d: DSCP Idx:%d, DSCP Ptr:%.8x, Done Bit:%d, PktLen:%d, PktAddr:%.8x(%.8x), Next Idx:%d\n", 
																idx, diPtr->dscpIdx, 
																(uint)diPtr->dscpPtr, 
																diPtr->dscpPtr->ctrl.done,
																diPtr->dscpPtr->ctrl.pkt_len, 
																(uint)diPtr->skb, 
																(uint)diPtr->dscpPtr->pkt_addr,
																diPtr->dscpPtr->next_idx) ;
			diPtr = diPtr->next ;
			idx++ ;
		}
	} while(diPtr!=NULL && diPtr!=gpQdmaPriv->txHeadPtr) ;

	
	return 0;
}


void dump_skb(struct sk_buff *skb)
{
	unsigned 	char tmp[80];
	unsigned char *p = skb->data;
	unsigned char *t = tmp;
	int i, n = 0;

    printk("skb data is below\n");
	if(skb == NULL)
		printk("skb is null");
	if(skb->data == NULL)
		printk("skb data is null");
	for (i = 0; i < skb->len; i++) {
		t += sprintf(t, "%02x ", *p++ & 0xff);
		if ((i & 0x0f) == 0x0f) {
			printk("%04x: %s\n", n, tmp);
			n += 16;
			t = tmp;
		}
	}
	if (i & 0x0f)
		printk("%04x: %s\n", n, tmp);
	printk("skb->len %d\n", skb->len);
}


void macSetMACCR(struct airoha_eth *eth, macAdapter_t *map_p)
{
	u32 reg;

	reg = (12<<GDM_JMB_LEN_SHIFT)  |
		(GDM_P_CPU<<GDM_UFRC_P_SHIFT) | (GDM_P_CPU<<GDM_BFRC_P_SHIFT) | 
		(GDM_P_CPU<<GDM_MFRC_P_SHIFT) | (GDM_P_CPU<<GDM_OFRC_P_SHIFT);
	write_reg_word(GDMA1_FWD_CFG(eth->base), reg);

	/* check if FPGA */
// 	if (isFPGA) {
// 		/* set 1us clock for FPGA */
// 		reg = read_reg_word(CR_CLK_CFG);
// 		reg &= ~(0x3f000000);
// 
// 		reg |= (0x31<<24);
// 
// 		write_reg_word(CR_CLK_CFG, reg);
// 	}
    //led_gpio_enable();
}

void macSetMacReg(struct airoha_eth *eth, macAdapter_t *mac_p)
{
	write_reg_word(GDMA1_MAC_ADRL(eth->base), mac_p->macAddr[2]<<24 | mac_p->macAddr[3]<<16 | \
                               mac_p->macAddr[4]<<8  | mac_p->macAddr[5]<<0);
	write_reg_word(GDMA1_MAC_ADRH(eth->base), mac_p->macAddr[0]<<8  | mac_p->macAddr[1]<<0);

	/* fill in switch's MAC address */
/*	//write_reg_word(GSW_SMACCR0, mac_p->macAddr[2]<<24 | mac_p->macAddr[3]<<16 | \
  //                             mac_p->macAddr[4]<<8  | mac_p->macAddr[5]<<0);
	//write_reg_word(GSW_SMACCR1, mac_p->macAddr[0]<<8  | mac_p->macAddr[1]<<0);*/
}

int macDrvRegInit(struct airoha_eth *eth, macAdapter_t *mac_p)
{
    // ----- setup interrupt mask ---
    //  ---- Setup HASH table------------------------------------------
    //  -----Setup Interrupt Timer-------------------------------------
    //  -----Setup AUTO polling timer----------------------------------
    //  ---- Setup DMA burst and arbitration---------------------------
    //  -----Setup DMA Descriptor Base Address Assign------------------    
    //  -----Setup MACCR-----------------------------------------------
    macSetMACCR(eth, mac_p);
    
    // --- setup MAC address ---
    macSetMacReg(eth, mac_p);      

   	//macSetGSW(mac_p);

    return 0;
}

void macGetMacAddr(struct airoha_eth *eth, macAdapter_t *mac_p, unsigned char *macAddr)
{
	int i;

	for (i = 0; i < 6; i++)
		mac_p->macAddr[i] = macAddr[i];
}


int qdmaEnableInt(uint base, uint bit, QDMA_InterruptIdx_T intIdx)
{
//	ulong flags=0 ;
	uint t=0 ;

	if( (intIdx < QDMA_INT1_ENABLE1) || (intIdx > QDMA_INT4_ENABLE2) )
	{
		printk("qdmaEnableInt: INT Index Error.\n");
		return -EINVAL;
	}

	if( (intIdx%2) == 1 )
	{
		t = IO_GREG(QDMA_CSR_INT_ENABLE1(base,((intIdx+1)>>1))) ;
		IO_SREG(QDMA_CSR_INT_ENABLE1(base,((intIdx+1)>>1)), (t|bit));
	}
	else
	{
		t = IO_GREG(QDMA_CSR_INT_ENABLE2(base,(intIdx>>1))) ;
		IO_SREG(QDMA_CSR_INT_ENABLE2(base,(intIdx>>1)), (t|bit));
	}

	return 0 ;
}

/******************************************************************************
******************************************************************************/
int qdmaDisableInt(uint base, uint bit, QDMA_InterruptIdx_T intIdx)
{
//	ulong flags=0 ;
	uint t=0 ;

	if( (intIdx < QDMA_INT1_ENABLE1) || (intIdx > QDMA_INT4_ENABLE2) )
	{
		printk("qdmaDisableInt: INT Index Error.\n");
		return -EINVAL;
	}

	if( (intIdx%2) == 1 )
	{
		t = IO_GREG(QDMA_CSR_INT_ENABLE1(base,((intIdx+1)>>1))) ;
		IO_SREG(QDMA_CSR_INT_ENABLE1(base,((intIdx+1)>>1)), (t&(~bit)));
	}
	else
	{
		t = IO_GREG(QDMA_CSR_INT_ENABLE2(base,(intIdx>>1))) ;
		IO_SREG(QDMA_CSR_INT_ENABLE2(base,(intIdx>>1)), (t&(~bit)));
	}

	return 0 ;
}

/******************************************************************************
******************************************************************************/
int qdmaSetIntMask(void * base, uint value, QDMA_InterruptIdx_T intIdx)
{
//	ulong flags=0 ;

	if( (intIdx < QDMA_INT1_ENABLE1) || (intIdx > QDMA_INT4_ENABLE2) )
	{
		printk("qdmaSetIntMask: INT Index Error\n");
		return -EINVAL;
	}

	if( (intIdx%2) == 1 )
	{
		IO_SREG(QDMA_CSR_INT_ENABLE1(base,((intIdx+1)>>1)), value) ;
	}
	else
	{
		IO_SREG(QDMA_CSR_INT_ENABLE2(base,(intIdx>>1)), value) ;
	}

	return 0 ;
}

/******************************************************************************
******************************************************************************/
int qdmaGetIntMask(uint base, QDMA_InterruptIdx_T intIdx)
{
	//ulong flags=0,
	u32 value=0 ;

	if( (intIdx < QDMA_INT1_ENABLE1) || (intIdx > QDMA_INT4_ENABLE2) )
	{
		printk("qdmaGetIntMask: INT Index Error\n");
		return -EINVAL;
	}

	if( (intIdx%2) == 1 )
	{
		value = IO_GREG(QDMA_CSR_INT_ENABLE1(base, ((intIdx+1)>>1))) ;
	}
	else
	{
		value = IO_GREG(QDMA_CSR_INT_ENABLE2(base, (intIdx>>1))) ;
	}

	return value ;
}

static int qdma_bm_push_tx_dscp(struct QDMA_DscpInfo_S *diPtr, int ringIdx) 
{
	if(diPtr->next != NULL) {
		QDMA_ERR("The TX DSCP is not return from tx used pool\n") ;
		return -1 ;
	}

	diPtr->skb = NULL ;
	if(!gpQdmaPriv->txHeadPtr) {
		gpQdmaPriv->txHeadPtr = diPtr ;
		gpQdmaPriv->txTailPtr = diPtr ;
	} else {
		gpQdmaPriv->txTailPtr->next = diPtr ;
		gpQdmaPriv->txTailPtr = gpQdmaPriv->txTailPtr->next ;
	}
	
	return 0 ;
}


/******************************************************************************
******************************************************************************/
static struct QDMA_DscpInfo_S *qdma_bm_pop_tx_dscp(void)
{
	struct QDMA_DscpInfo_S *diPtr ;
//	ulong flags ;
	
	diPtr = gpQdmaPriv->txHeadPtr ;
	if(gpQdmaPriv->txHeadPtr == gpQdmaPriv->txTailPtr) {
		gpQdmaPriv->txHeadPtr = NULL ;
		gpQdmaPriv->txTailPtr = NULL ;
	} else {
		gpQdmaPriv->txHeadPtr = gpQdmaPriv->txHeadPtr->next ;
	}

	if(diPtr) {
		diPtr->next = NULL ;
	}
	
	return diPtr ;
}

static void qdma_bm_add_rx_dscp(struct QDMA_DscpInfo_S *diPtr) 
{
	if(!gpQdmaPriv->rxStartPtr) {
		gpQdmaPriv->rxStartPtr = diPtr ;
		diPtr->next = gpQdmaPriv->rxStartPtr ;
	} else {
		diPtr->next = gpQdmaPriv->rxStartPtr->next ;
		gpQdmaPriv->rxStartPtr->next = diPtr ;
		gpQdmaPriv->rxStartPtr = diPtr ;
	}
}


static struct QDMA_DscpInfo_S *qdma_bm_get_unused_rx_dscp(void)
{
	struct QDMA_DscpInfo_S *diPtr = NULL ;

	
	if(gpQdmaPriv->rxStartPtr) {
		if(!gpQdmaPriv->rxEndPtr) {
			diPtr = gpQdmaPriv->rxStartPtr ;
			gpQdmaPriv->rxEndPtr = diPtr ;
		} else if(gpQdmaPriv->rxEndPtr->next != gpQdmaPriv->rxStartPtr) {
			diPtr = gpQdmaPriv->rxEndPtr->next ;
			gpQdmaPriv->rxEndPtr = diPtr ; 
		}
	} 

	return diPtr ;
}

int qdma_bm_hook_receive_buffer(struct airoha_eth *eth, struct sk_buff *skb, int ringIdx)
{
	struct QDMA_DscpInfo_S *pNewDscpInfo ;
	QDMA_DMA_DSCP_T *pRxDscp ;
	dma_addr_t dmaPktAddr ;
	void * base = gpQdmaPriv->csrBaseAddr ;
	int ret = 0 ;
//	unsigned long	addr, len;

	pNewDscpInfo = qdma_bm_get_unused_rx_dscp() ;
	if(pNewDscpInfo == NULL) {
		QDMA_ERR("There is not any free RX DSCP.\n") ; 
		gpQdmaPriv->counters.noRxDscps++ ;
		return -1 ;
	}
	
#ifdef CONFIG_RX_2B_OFFSET
// 	QDMA_MSG( "Adjust the skb->tail location for net IP alignment\n") ;
// 	if(((uint)skb->data & 7) != 0) {
// 		//prom_printf("address not align 8\n");
// 	}
	skb_reserve(skb, NET_IP_ALIGN) ;
	dmaPktAddr = virt_to_phys((void *)((uint)skb->data - NET_IP_ALIGN));

	// NOTE Unknown size maybe skb_tailroom() or HWFWD_PAYLOAD_SIZE_2K (MAX_PKT_LENS) or pRxDscp->ctrl.pkt_len?
// 	dmaPktAddr = dma_map_single(eth->dma_dev, skb->data-NET_IP_ALIGN, MAX_PKT_LENS, DMA_FROM_DEVICE);
// 	if (unlikely(dma_mapping_error(eth->dma_dev, dmaPktAddr)))
// 		return -ENOMEM;
#else
	dmaPktAddr = virt_to_phys((void *)((uint)skb->data));
#endif /* CONFIG_RX_2B_OFFSET */
	
	
	pRxDscp = gpQdmaPriv->rxUsingPtr->dscpPtr ;
	pRxDscp->msg[0] = 0;
	pRxDscp->msg[1] = 0;
	pRxDscp->msg[2] = 0;
	pRxDscp->msg[3] = 0;
	pRxDscp->pkt_addr = dmaPktAddr ;
	pRxDscp->next_idx = pNewDscpInfo->dscpIdx ;
	pRxDscp->ctrl.pkt_len = 1518;
	pRxDscp->ctrl.done = 0;
	

// 	QDMA_MSG("Hook RX DSCP to RXDMA. RX_CPU_IDX:%.8x, RX_NULL_IDX:%.8x\n", gpQdmaPriv->rxUsingPtr->dscpIdx, pNewDscpInfo->dscpIdx);
// 	QDMA_MSG("RXDSCP(%x): DONE:%d, PKT:%.8x, PKTLEN:%d, NEXT_IDX:%d\n", 
// 													pRxDscp,
// 													(uint)pRxDscp->ctrl.done, 
// 													(uint)pRxDscp->pkt_addr,
// 													(uint)pRxDscp->ctrl.pkt_len,
// 													(uint)pRxDscp->next_idx) ;
													
	gpQdmaPriv->rxUsingPtr->skb = skb;
	gpQdmaPriv->rxUsingPtr = pNewDscpInfo;
	gpQdmaPriv->counters.rxCounts++;


//	addr = (unsigned long )&mac_p->macMemPool_p->descrPool[CONFIG_TX0_DSCP_NUM*CONFIG_TX0_DSCP_SIZE] & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	len = (CONFIG_RX0_DSCP_NUM*CONFIG_RX0_DSCP_SIZE + 2 * CONFIG_SYS_CACHELINE_SIZE) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	flush_dcache_range(addr, addr + len );

//	addr = (unsigned long )pRxDscp->pkt_addr & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	len = (1518 + 2 * CONFIG_SYS_CACHELINE_SIZE) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	flush_dcache_range(addr, addr + len);
v7_flush_kern_cache_all();
	/* Setting DMA Rx Descriptor Register */
	qdmaSetRxCpuIdx(base, ringIdx, pNewDscpInfo->dscpIdx);

	return ret;
}


static int qdma_prepare_rx_buffer(struct airoha_eth *eth)
{
	struct sk_buff *skb = NULL ;

	//skb = alloc_skb(2000);
	skb = alloc_skb(2000, GFP_KERNEL);
	if(skb == NULL) {
		return -1;
	}

	if(qdma_bm_hook_receive_buffer(eth, skb, 0) != 0) {
		kfree_skb(skb);
		return -1;
	}
	
	return 0 ;
}


int qdma_has_free_rxdscp(void)
{
	return (gpQdmaPriv->rxEndPtr->next != gpQdmaPriv->rxStartPtr) ;
}

static int qdma_bm_dscp_init(struct airoha_eth *eth)
{
	struct QDMA_DscpInfo_S *diPtr ;
	QDMA_DMA_DSCP_T *pHwDscp ;
	dma_addr_t dscpDmaAddr, irqDmaAddr, hwFwdDmaAddr ;
	dma_addr_t hwFwdBuffAddr ;
	uint virAddr=0;
	void __iomem			*dscpBaseAddr;
	uint hwTotalDscpSize, hwTotalMsgSize, hwTotalPktSize ;
	uint i;
	void * base;
//	int ringIdx=0 ;
	uint txDscpNum ;
	uint rxDscpNum ;
	uint hwDscpNum ;
	uint irqDepth ;
	uint hwFwdPktLen ;
	int flag=0, cnt;
//	unsigned long   addr, len;
	uint lmgrInitCfg = 0 ;

	dev_err(eth->dev, "qdma_bm_dscp_init()\n");

	base = gpQdmaPriv->csrBaseAddr ;
	txDscpNum = gpQdmaPriv->txDscpNum ;
	rxDscpNum = gpQdmaPriv->rxDscpNum ;
	hwDscpNum = gpQdmaPriv->hwFwdDscpNum ;
	irqDepth = gpQdmaPriv->irqDepth ;
	hwFwdPktLen = gpQdmaPriv->hwPktSize ;	

	
	/******************************************
	* Allocate descriptor DMA memory          *
	*******************************************/
//	dscpBaseAddr = K0_TO_K1((uint)&mac_p->macMemPool_p->descrPool[0]) ;
//	dscpDmaAddr = K1_TO_PHY(dscpBaseAddr);

// NOTE one ring seems to be shared between RX and TX

	dev_err(eth->dev, "Allocate descriptor DMA memory dma_alloc_coherent()\n");

	dscpBaseAddr = dma_alloc_coherent(eth->dma_dev, sizeof(QDMA_DMA_DSCP_T)*(txDscpNum + rxDscpNum), &dscpDmaAddr, GFP_KERNEL);

	if (dscpBaseAddr == NULL) {
		dev_err(eth->dev, "DMA memory allocation for dscpBaseAddr failed\n");
		return -ENOMEM;
	}

	//Set the TX_DSCP_BASE and RX_DSCP_BASE address
// 	addr = (unsigned long )&mac_p->macMemPool_p->descrPool[0] & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
// 	len = (CONFIG_TX0_DSCP_NUM*CONFIG_TX0_DSCP_SIZE + 2 * CONFIG_SYS_CACHELINE_SIZE) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
// 	flush_dcache_range(addr, addr + len );
	v7_flush_kern_cache_all();
	dev_err(eth->dev, "qdmaSetTxDscpBase()\n");
	qdmaSetTxDscpBase(base, RING_IDX_0, dscpDmaAddr) ;

// 	addr = (unsigned long )&mac_p->macMemPool_p->descrPool[CONFIG_TX0_DSCP_NUM*CONFIG_TX0_DSCP_SIZE] & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
// 	len = (CONFIG_RX0_DSCP_NUM*CONFIG_RX0_DSCP_SIZE + 2 * CONFIG_SYS_CACHELINE_SIZE) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
// 	flush_dcache_range(addr, addr + len );
	dev_err(eth->dev, "qdmaSetRxDscpBase()\n");
v7_flush_kern_cache_all();
	qdmaSetRxDscpBase(base, RING_IDX_0, (dscpDmaAddr + sizeof(QDMA_DMA_DSCP_T)*(txDscpNum))) ;	

	qdmaSetRxRingSize(base, RING_IDX_0, rxDscpNum);
	qdmaSetRxRingThrh(base, RING_IDX_0, 0);

	/******************************************
	* Allocate memory for IRQ queue           *
	******************************************/
	if(irqDepth) {

//		gpQdmaPriv->irqQueueAddr = K0_TO_K1((dscpBaseAddr + sizeof(QDMA_DMA_DSCP_T)*(txDscpNum + rxDscpNum))) ;
//		irqDmaAddr = K1_TO_PHY(gpQdmaPriv->irqQueueAddr);

		dev_err(eth->dev, "Allocate memory for IRQ queue dma_alloc_coherent2()\n");

		gpQdmaPriv->irqQueueAddr = dma_alloc_coherent(eth->dma_dev, irqDepth<<2, &irqDmaAddr, GFP_KERNEL);
		if (gpQdmaPriv->irqQueueAddr == NULL) {
			dev_err(eth->dev, "DMA memory allocation for gpQdmaPriv->irqQueueAddr failed\n");
			return -ENOMEM;
		}
		memset((void *)gpQdmaPriv->irqQueueAddr, CONFIG_IRQ_DEF_VALUE, irqDepth<<2) ;

//		addr = (unsigned long )&mac_p->macMemPool_p->descrPool[DESC_TOTAL_SIZE] & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//		len = ((CONFIG_IRQ_DEPTH<<2)+ 2 * CONFIG_SYS_CACHELINE_SIZE) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//		flush_dcache_range(addr, addr + len );
		//flush_cache_range(gpQdmaPriv->irqQueueAddr, irqDepth<<2);
		v7_flush_kern_cache_all();
		
		/* Setting the IRQ queue information to QDMA register */
		qdmaSetIrqBase(base, irqDmaAddr) ;
		qdmaSetIrqDepth(base, irqDepth) ;
	}
	/***************************************************
	* Allocate memory for TX/RX DSCP Information node  *
	****************************************************/
//	gpQdmaPriv->dscpInfoAddr = &DummydscpInfoAddr ;
//	memset(gpQdmaPriv->dscpInfoAddr, 0, DESC_INFO_SIZE);


	gpQdmaPriv->dscpInfoAddr = kzalloc(DESC_INFO_SIZE, GFP_KERNEL);
	dev_err(eth->dev, "kzalloc() dscpInfoAddr = %x\n", (void*)gpQdmaPriv->dscpInfoAddr);

	if (gpQdmaPriv->dscpInfoAddr == NULL) {
		dev_err(eth->dev, "memory allocation for gpQdmaPriv->dscpInfoAddr failed\n");
		return -ENOMEM;
	}
	
	gpQdmaPriv->txBaseAddr = gpQdmaPriv->dscpInfoAddr;
	gpQdmaPriv->rxBaseAddr = gpQdmaPriv->dscpInfoAddr + sizeof(struct QDMA_DscpInfo_S)*txDscpNum;

	dev_err(eth->dev, "Create unused tx descriptor\n");

	//Create unused tx descriptor link list and using rx descriptor ring
	for(i=0 ; i<(txDscpNum + rxDscpNum) ; i++)
	{
		diPtr = (struct QDMA_DscpInfo_S *)gpQdmaPriv->dscpInfoAddr + i ;
		diPtr->dscpPtr = (QDMA_DMA_DSCP_T *)dscpBaseAddr + i ;
		
		if(i < txDscpNum) {
			diPtr->dscpIdx = i ;
			diPtr->next = NULL ;
			dev_err(eth->dev, "[%d] qdma_bm_push_tx_dscp(%d) = %x diPtr->dscpIdx = %x\n", i, i, (void*)diPtr, (void*)diPtr->dscpPtr);

			qdma_bm_push_tx_dscp(diPtr, 0) ;
		} else  {
			diPtr->dscpIdx = i - txDscpNum ;
			diPtr->next = NULL ;
			dev_err(eth->dev, "[%d] qdma_bm_add_rx_dscp(%d) = %x diPtr->dscpIdx = %x\n", i, i-txDscpNum, (void*)diPtr, (void*)diPtr->dscpPtr);

			qdma_bm_add_rx_dscp(diPtr) ;
		}
	}	
	//qdma_bm_dump_dscp();
	dev_err(eth->dev, "Initialization first DSCP for Tx0 DMA\n");

	/***************************************************
	* Initialization first DSCP for Tx0 DMA              *
	****************************************************/
	diPtr = qdma_bm_pop_tx_dscp() ;
	if(!diPtr) {
		dev_err(eth->dev,"There is not any free TX DSCP.\n") ; 
		return -ENOSR ;
	}
	gpQdmaPriv->txUsingPtr = diPtr ;
	qdmaSetTxCpuIdx(base, RING_IDX_0, diPtr->dscpIdx) ;
	qdmaSetTxDmaIdx(base, RING_IDX_0, diPtr->dscpIdx) ;
	
	/***************************************************
	* Initialization first DSCP for Rx0 DMA              *
	****************************************************/
	dev_err(eth->dev, "qdma_bm_get_unused_rx_dscp\n");

	diPtr = qdma_bm_get_unused_rx_dscp() ;
	if(diPtr == NULL) {
		dev_err(eth->dev,"There is not any free RX DSCP.\n") ;
		return -ENOSR ;
	} 
	gpQdmaPriv->rxUsingPtr = diPtr ;
	qdmaSetRxCpuIdx(base, RING_IDX_0, diPtr->dscpIdx) ;
	qdmaSetRxDmaIdx(base, RING_IDX_0, diPtr->dscpIdx) ;

	/***************************************************
	* Initialization packets for Rx DMA              *
	****************************************************/
	dev_err(eth->dev, "qdma_prepare_rx_buffer\n");

	do {
		if(qdma_prepare_rx_buffer(eth) != 0)
		{
			dev_err(eth->dev, "qdma_prepare_rx_buffer failed\n");
			break ;
		}
	} while(qdma_has_free_rxdscp()) ;	

	/***************************************************
	* Initialization DSCP for hardware forwarding      *
	****************************************************/
	dev_err(eth->dev, "Initialization DSCP for hardware forwarding\n");

	if(hwDscpNum) {

		hwTotalDscpSize = sizeof(QDMA_HWFWD_DMA_DSCP_T) * hwDscpNum ;

		hwTotalPktSize = hwFwdPktLen * hwDscpNum ;

		gpQdmaPriv->hwFwdPayloadSize = hwFwdPktLen;

// 		gpQdmaPriv->hwFwdBaseAddr = K0_TO_K1(gpQdmaPriv->irqQueueAddr + (CONFIG_IRQ_DEPTH<<2) );
// 		hwFwdDmaAddr = K1_TO_PHY(gpQdmaPriv->hwFwdBaseAddr);

		gpQdmaPriv->hwFwdBaseAddr = dma_alloc_coherent(eth->dma_dev, hwTotalDscpSize, &hwFwdDmaAddr, GFP_KERNEL);

// 		gpQdmaPriv->hwFwdBuffAddr = K0_TO_K1(gpQdmaPriv->hwFwdBaseAddr + hwTotalDscpSize) ;
// 		hwFwdBuffAddr = K1_TO_PHY(gpQdmaPriv->hwFwdBuffAddr);

 		gpQdmaPriv->hwFwdBuffAddr = dma_alloc_coherent(eth->dma_dev, hwTotalPktSize, &hwFwdBuffAddr, GFP_KERNEL);

//		hwFwdBuffAddr = (448-16) << 20;
//		gpQdmaPriv->hwFwdBuffAddr = (uint)(ioremap(hwFwdBuffAddr, hwTotalPktSize));

		if (gpQdmaPriv->hwFwdBuffAddr)
			dev_err(eth->dev, "gpQdmaPriv->hwFwdBuffAddr = dma_alloc_coherent() failed\n");

		dev_err(eth->dev, "gpQdmaPriv->hwFwdBaseAddr %x\n", (void*)gpQdmaPriv->hwFwdBaseAddr);
		dev_err(eth->dev, "gpQdmaPriv->hwFwdBuffAddr %x\n", (void*)gpQdmaPriv->hwFwdBuffAddr);
		dev_err(eth->dev, "hwFwdDmaAddr %x\n", (void*)hwFwdDmaAddr);
		dev_err(eth->dev, "hwFwdBuffAddr %x\n", (void*)hwFwdBuffAddr);

		qdmaSetHwDscpBase(base, hwFwdDmaAddr);
		qdmaSetHwBuffBase(base, hwFwdBuffAddr);

		qdmaSetHwPayloadSize(base, HWFWD_PAYLOAD_SIZE_2K);
		qdmaSetHwLowThrshld(base, HWFWD_LOW_THRESHOLD);

//#if defined(TCSUPPORT_CPU_EN7580) 
		dev_err(eth->dev, "qdmaGetHwInitCfg\n");
		lmgrInitCfg = qdmaGetHwInitCfg(base) ;
		lmgrInitCfg = (lmgrInitCfg | LMGR_INIT_START | hwDscpNum) ; /*set DSCP in DRAM*/
		qdmaSetHwInitCfg(base, lmgrInitCfg) ;
//#else
// 		qdmaSetHwDscpNum(base, hwDscpNum);
// 		qdmaSetHWInitStart(base);
// #endif

		flag = 0 ;

		cnt = 1000;
		while((cnt--) > 0) {		
			if(qdmaGetHWInitStart(base) == 0) {
				flag=1;
				break;
			}
			
		}

		if(flag == 0) {
			dev_err(eth->dev,"hw_fwd init fail!\n") ;
			return -1;
		}
	}
	dev_err(eth->dev, "qdma_bm_dscp_init done\n");

	return 0 ;
}



int qdma_bm_receive_packets(struct airoha_eth *eth, uint maxPkts, int ringIdx)
{
	QDMA_DMA_DSCP_T rxDscp ;
	struct QDMA_DscpInfo_S dscpInfo ;
	uint cnt = maxPkts;
	uint pktCount = 0;
	int retValue = 0 , g;
	struct sk_buff *newSkb = NULL;
	struct sk_buff *rxSkb = NULL;
	void* base = gpQdmaPriv->csrBaseAddr;
	struct ethhdr *eth_hdr = NULL;
	unsigned long	addr, len;

	
#if 1
	do {

// 		addr = (unsigned long )&mac_p->macMemPool_p->descrPool[CONFIG_TX0_DSCP_NUM*CONFIG_TX0_DSCP_SIZE] & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
// 		len = (CONFIG_RX0_DSCP_NUM*CONFIG_RX0_DSCP_SIZE + 2 * CONFIG_SYS_CACHELINE_SIZE) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
// 		invalidate_dcache_range(addr, addr + len );
		v7_flush_kern_cache_all();
		if(!gpQdmaPriv->rxStartPtr || gpQdmaPriv->rxStartPtr==gpQdmaPriv->rxEndPtr || !gpQdmaPriv->rxStartPtr->dscpPtr->ctrl.done)
		{
			goto ret ;
		}

		//qdma_bm_dump_dscp();
		memcpy(&rxDscp, gpQdmaPriv->rxStartPtr->dscpPtr, sizeof(QDMA_DMA_DSCP_T)) ;
		memcpy(&dscpInfo, gpQdmaPriv->rxStartPtr, sizeof(struct QDMA_DscpInfo_S)) ;
		
		gpQdmaPriv->rxStartPtr = gpQdmaPriv->rxStartPtr->next ;
		
		pktCount++ ;
		/* check DSCP cotent */
		if(!rxDscp.pkt_addr || !rxDscp.ctrl.pkt_len)
		{
			QDMA_ERR("The content of the RX DSCP is incorrect.\n") ;
			gpQdmaPriv->counters.rxDscpIncorrect++ ; 
			break ;
		}
		
		newSkb = alloc_skb(2000, GFP_KERNEL);
		if(newSkb == NULL) {
			newSkb = dscpInfo.skb;
			QDMA_ERR("\nalloc fail\n");
			goto next;
		}
		dscpInfo.skb->len = rxDscp.ctrl.pkt_len;

// 		addr = (unsigned long)dscpInfo.skb->data & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
// 		len = (rxDscp.ctrl.pkt_len + 2 * CONFIG_SYS_CACHELINE_SIZE ) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
// 		invalidate_dcache_range(addr, addr + len);
		dump_skb(dscpInfo.skb);
		//eth_rcv(dscpInfo.skb);

		kfree_skb(dscpInfo.skb);
next:
		if (qdma_has_free_rxdscp())
		{
			qdma_bm_hook_receive_buffer(eth, newSkb, 0);
		} else {
			QDMA_ERR("\nRX Error: no available QDMA RX descritor\n");
			gpQdmaPriv->counters.noRxDscps++ ;
			kfree_skb(newSkb);
		}
		

	} while((!maxPkts) || (--cnt)) ;

	return pktCount ;

ret:

	return pktCount ;
#else
	int i = 0;
	for(i = 0; i < gpQdmaPriv->rxDscpNum; i ++){
		memcpy(&rxDscp, gpQdmaPriv->rxStartPtr->dscpPtr, sizeof(QDMA_DMA_DSCP_T)) ;
		memcpy(&dscpInfo, gpQdmaPriv->rxStartPtr, sizeof(struct QDMA_DscpInfo_S)) ;
		gpQdmaPriv->rxStartPtr = gpQdmaPriv->rxStartPtr->next;
		if(gpQdmaPriv->rxStartPtr == NULL)
		{
			printf("gpQdmaPriv->rxStartPtr is null");

		return 0;
		}

		if(dscpInfo.skb == NULL)
			printf("skb is null");
		else{
			dscpInfo.skb->len = rxDscp.ctrl.pkt_len;
			dump_skb(dscpInfo.skb);

		}
		printf("idx is %d dscp idx is %daddr is\n", i, dscpInfo.dscpIdx);


	}
	return 0;

#endif
done:
// 	tftp_start = 0;
// 	NetBootFileXferSize = 0;
// 	net_set_state(NETLOOP_CONTINUE);
// 	net_set_udp_handler(NULL);
// 	net_set_arp_handler(NULL);
// 	NetSetTimeout(0, NULL);

	return 0;

}



int qdma_dev_init(struct airoha_eth *eth) 
{
	void * base = gpQdmaPriv->csrBaseAddr ;
	uint i=0, glbCfg=0, intEnable=0 ;
	uint int_Enable1=0, int_Enable2=0 ;

	dev_err(eth->dev, "qdma_dev_init\n");

	qdmaClearIntStatus1(base, 0xFFFFFFFF) ;
	qdmaClearIntStatus2(base, 0xFFFFFFFF) ;
	/********************************************
	* enable/disable the qdma interrupt         *
	*********************************************/
	dev_err(eth->dev, "enable/disable the qdma interrupt\n");

	/*only enable rx-0 INT*/
	int_Enable1 = 0 ;
	int_Enable2 = INT_MASK_RX_DONE ;
	qdmaSetIntMask(base, int_Enable1, QDMA_INT1_ENABLE1) ;
	qdmaSetIntMask(base, int_Enable2, QDMA_INT1_ENABLE2) ;
	
	/********************************************
	* Setting the global register               *
	*********************************************/
// #ifndef TCSUPPORT_LITTLE_ENDIAN
// 	glbCfg |= (GLB_CFG_DSCP_BYTE_SWAP | GLB_CFG_PAYLOAD_BYTE_SWAP | GLB_CFG_MSG_WORD_SWAP | ((VAL_BST_32_DWARD<<GLB_CFG_BST_SE_SHIFT)&GLB_CFG_BST_SE_MASK)) ;
// #else
	glbCfg |= (GLB_CFG_PAYLOAD_BYTE_SWAP | GLB_CFG_MSG_WORD_SWAP | ((VAL_BST_32_DWARD<<GLB_CFG_BST_SE_SHIFT)&GLB_CFG_BST_SE_MASK)) ;
// #endif

	glbCfg |= (PREFER_TX1_FWD_TX0<<GLB_CFG_DMA_PREFERENCE_SHIFT)&GLB_CFG_DMA_PREFERENCE_MASK;

	if(gpQdmaPriv->irqDepth) {
		glbCfg |= GLB_CFG_IRQ_EN ;
	}
	
#ifdef CONFIG_RX_2B_OFFSET
	glbCfg |= GLB_CFG_RX_2B_OFFSET ;
#endif /* CONFIG_RX_2B_OFFSET */

#ifdef CONFIG_TX_WB_DONE
	glbCfg |= GLB_CFG_TX_WB_DONE ;
#endif /* GLB_CFG_TX_WB_DONE */
	dev_err(eth->dev, "qdmaSetGlbCfg\n");

	qdmaSetGlbCfg(base, glbCfg) ;
	for(i=0; i<RX_RING_NUM; i++) {
		write_reg_word(QDMA_CSR_RX_DELAY_INT_CFG(base, i), 0);
	}
	return 0 ;
}

int qdma_bm_transmit_done(int amount) 
{
	QDMA_DMA_DSCP_T txDscp ;
	int ret = 0 ;
	struct QDMA_DscpInfo_S *diPtr ;
	void* base = gpQdmaPriv->csrBaseAddr ;
	uint entryLen, headIdx, irqValue=0, irqDepth=IRQ_DEPTH ;
	uint *irqPtr ;
	int i=0, j=0, idx=0, ringIdx=0 ;
	uint RETRY=3 ;
	uint irqStatus ;
	void *msgPtr;

	irqStatus = qdmaGetIrqStatus(base) ;
	headIdx = (irqStatus & IRQ_STATUS_HEAD_IDX_MASK) >> IRQ_STATUS_HEAD_IDX_SHIFT ;
	entryLen = (irqStatus & IRQ_STATUS_ENTRY_LEN_MASK) >> IRQ_STATUS_ENTRY_LEN_SHIFT ;
	if(entryLen == 0) {
		QDMA_MSG("qdma_bm_transmit_done-111111\n") ;
		goto out2 ;
	}

	entryLen = (amount && amount<entryLen) ? amount : entryLen ;
	for(i=0 ; i<entryLen ; i++) {
		irqPtr = (uint *)gpQdmaPriv->irqQueueAddr + ((headIdx+i)%irqDepth) ;
		
		RETRY = 3 ;
		while(RETRY--) {
			irqValue = *irqPtr ;
			if(irqValue == CONFIG_IRQ_DEF_VALUE) {
				printk("There is no data available in IRQ queue. irq value:%.8x, irq ptr:%.8x TIMEs:%d\n", (uint)irqValue, (uint)irqPtr, RETRY) ;
				if(RETRY <= 0) {
					gpQdmaPriv->counters.IrqQueueAsynchronous++ ;
					ret = -1 ;
					goto out1 ;
				}
			} else {
				*irqPtr = CONFIG_IRQ_DEF_VALUE ;
				break ;
			}
		}
		
		idx = (irqValue & IRQ_CFG_IDX_MASK) ;
		ringIdx = (irqValue & IRQ_CFG_RINGIDX_MASK) >> IRQ_CFG_RINGIDX_SHIFT;
		if(idx<0 || idx>=gpQdmaPriv->txDscpNum) {
			printk("The TX DSCP index %d is invalid.\n", idx) ;
			gpQdmaPriv->counters.txIrqQueueIdxErrs++ ;
			ret = -1;
			continue ;
		}
			
		diPtr = (struct QDMA_DscpInfo_S *)gpQdmaPriv->txBaseAddr + idx ;
		if(diPtr->dscpIdx!=idx || diPtr->next!=NULL) {
			printk("The content of the TX DSCP_INFO(%.8x) is incorrect. ENTRY_LEN:%d, HEAD_IDX:%d, IRQ_VALUE:%.8x.\n", (uint)diPtr, entryLen, headIdx, irqValue) ;
			gpQdmaPriv->counters.txDscpIncorrect++ ;
			ret = -1;
			continue ;
		}
		
		msgPtr = (void *)txDscp.msg ;
		if(msgPtr)
			memset(msgPtr, 0, QDMA_TX_DSCP_MSG_LENS);
		kfree_skb(diPtr->skb);
		
		qdma_bm_push_tx_dscp(diPtr, 0) ;

	}
	printk("qdma_bm_transmit_done-000000\n") ;

out1:
	for(j=0 ; j<(i>>7) ; j++) {
		qdmaSetIrqClearLen(base, 0x80) ;
	}
	qdmaSetIrqClearLen(base, (i&0x7F)) ;

out2:
	return ret ;
}


void macSetGSW(struct airoha_eth *eth, macAdapter_t *mac_p)
{
	u32 reg;
    int phy_add_start=0,phy_add_end=0,phy_add;
	/* set port 6 as 1Gbps, FC on */
	reg = (IPG_CFG_SHORT<<IPG_CFG_PN_SHIFT) | MAC_MODE_PN | FORCE_MODE_PN | 
		MAC_TX_EN_PN | MAC_RX_EN_PN | BKOFF_EN_PN | BACKPR_EN_PN | 
		ENABLE_RX_FC_PN | ENABLE_TX_FC_PN | (PN_SPEED_1000M<<FORCE_SPD_PN_SHIFT) | 
		FORCE_DPX_PN | FORCE_LNK_PN;
	write_reg_word(eth->esw + 0x3600, reg);

	/* set cpu port as port 6 */
	reg = (0x40<<MFC_BC_FFP_SHIFT) | (0x40<<MFC_UNM_FFP_SHIFT) | (0x40<<MFC_UNU_FFP_SHIFT) |
			MFC_CPU_EN	| (6<<MFC_CPU_PORT_SHIFT);
		write_reg_word(eth->esw + 0x10, reg);

		/* Sideband signal error for Port 3, which need the auto polling */
		write_reg_word(eth->esw+0x7018, 0x7f7f8c08);
}


int setup_once = 1;
void switch_setup(struct airoha_eth *eth, macAdapter_t *mac_p) {
	
	u32 reg;
	dev_err(eth->dev, "switch_setup: reset\n");
	/* reset ethernet switch */
// 	reg = ioread32(eth->rst_2);
// 	reg |= (ESW_RST);
// 	iowrite32(reg, eth->rst_2);
// 
// 	msleep(20);
// 
// 	/* de-assert ethernet switch */
// 	reg = ioread32(eth->rst_2);
// 	reg &= ~(ESW_RST);
// 	iowrite32(reg, eth->rst_2);
// 
// 	/* add delay time to prevent switch reg I/O hang */
//     msleep(20);
		
		macSetGSW(eth, mac_p);

	/* fill in switch's MAC address */
	write_reg_word(eth->esw + 0x30e4, mac_p->macAddr[2]<<24 | mac_p->macAddr[3]<<16 | \
                               mac_p->macAddr[4]<<8  | mac_p->macAddr[5]<<0);
	write_reg_word(eth->esw + 0x30e8, mac_p->macAddr[0]<<8  | mac_p->macAddr[1]<<0);
	
	setup_once = 0;
}

int qdma_bm_transmit_packet(struct airoha_eth *eth, struct sk_buff *skb, int ringIdx, uint msg0, uint msg1)
{
	struct QDMA_DscpInfo_S *pNewDscpInfo ;
	QDMA_DMA_DSCP_T *pTxDscp ;
	void* base = gpQdmaPriv->csrBaseAddr ;
	int ret = 0 ;
	unsigned long   addr, len;

	if(!skb || skb->len<=0 || skb->len>CONFIG_MAX_PKT_LENS) 
	{
		printk("The input arguments are wrong, skb:%.8x, skbLen:%d.\n", (uint)skb, skb->len) ; 
        return -1;
	}

	if (setup_once) {
		switch_setup(eth, mac_p);
	}
	
	/* recycle TX DSCP when send packets in tx polling mode */
	
	if(qdmaGetIrqEntryLen(base) >= QDMA_TX_THRESHOLD) { 
		qdma_bm_transmit_done(0) ;
	}

	/* Get unused TX DSCP from TX unused DSCP link list */	
	pNewDscpInfo = qdma_bm_pop_tx_dscp() ;
	if(pNewDscpInfo == NULL) {
		gpQdmaPriv->counters.noTxDscps++ ;
		QDMA_ERR("pNewDscpInfo is NULL\n") ; 
		return -1 ;
	}
	pTxDscp = gpQdmaPriv->txUsingPtr->dscpPtr;
	pTxDscp->msg[0] = msg0;
	pTxDscp->msg[1] = msg1;
	pTxDscp->next_idx = pNewDscpInfo->dscpIdx ;
//	pTxDscp->pkt_addr = virt_to_phys(skb->data);
	pTxDscp->pkt_addr = dma_map_single(eth->dev, skb->data, skb->len, DMA_TO_DEVICE) ;

	if (!pTxDscp->pkt_addr)
		printk("pTxDscp->pkt_addr NULL");
	pTxDscp->ctrl.pkt_len = skb->len /* pktLen */; 
//	pTxDscp->ctrl.nls = 0 ;
//	pTxDscp->ctrl.drop_pkt = 0 ;
	pTxDscp->ctrl.done = 0 ;

// 	printk("Hook TX DSCP to TXDMA. TX_CPU_IDX:%d, TX_NULL_IDX:%d\n", gpQdmaPriv->txUsingPtr->dscpIdx, pNewDscpInfo->dscpIdx) ;
// 	printk("TXDSCP: DONE:%d, PKT:%.8x, PKTLEN:%d, NEXT_IDX:%d, loopcnt:%d\n", 
// 																(uint)pTxDscp->ctrl.done, 
// 																(uint)pTxDscp->pkt_addr,
// 																(uint)pTxDscp->ctrl.pkt_len,
// 																(uint)pTxDscp->next_idx,
// 																(uint)pTxDscp->msg[1]>>24) ;


	gpQdmaPriv->txUsingPtr->skb = skb ;


	//dump_skb(skb);
	gpQdmaPriv->txUsingPtr = pNewDscpInfo ;


//	addr = (unsigned long)(pTxDscp->pkt_addr) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	len = ((unsigned long)(pTxDscp->ctrl.pkt_len) + 2 * CONFIG_SYS_CACHELINE_SIZE ) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	flush_dcache_range(addr, addr + len);
		//flush_cache_range(pTxDscp->pkt_addr, sizeof(pTxDscp->pkt_addr))
//v7_flush_kern_cache_all();
	
//	addr = (unsigned long )&mac_p->macMemPool_p->descrPool[0] & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	len = (CONFIG_TX0_DSCP_NUM*CONFIG_TX0_DSCP_SIZE  + 2 * CONFIG_SYS_CACHELINE_SIZE) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	flush_dcache_range(addr, addr + len );
		//flush_cache_range(gpQdmaPriv->dscpInfoAddr, CONFIG_TX0_DSCP_NUM*CONFIG_TX0_DSCP_SIZE)
//v7_flush_kern_cache_all();
	//need for irq recycle
//	addr = (unsigned long )&mac_p->macMemPool_p->descrPool[DESC_TOTAL_SIZE] & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	len = ((CONFIG_IRQ_DEPTH<<2)+ 2 * CONFIG_SYS_CACHELINE_SIZE) & ~(CONFIG_SYS_CACHELINE_SIZE - 1);
//	flush_dcache_range(addr, addr + len );
//		flush_cache_range(pTxDscp, sizeof(QDMA_DMA_DSCP_T))
v7_flush_kern_cache_all();

	qdmaSetTxCpuIdx(base, ringIdx, pNewDscpInfo->dscpIdx) ;
	gpQdmaPriv->counters.txCounts++ ;


// 	printk("RingIdx:%d, GLG:%.8x, IRQStatus:%.8x, CSR info: RX_CPU_IDX:%d, RX_DMA_IDX:%d, TX_CPU_IDX:%d, TX_DMA_IDX:%d\n", 
// 																ringIdx, 
// 																qdmaGetGlbCfg(base), 
// 																qdmaGetIrqStatus(base), 
// 																qdmaGetRxCpuIdx(base, ringIdx), 
// 																qdmaGetRxDmaIdx(base, ringIdx), 
// 																qdmaGetTxCpuIdx(base, ringIdx), 
// 																qdmaGetTxDmaIdx(base, ringIdx)) ;

	return ret ;
}



int tc3162_eth_send(struct airoha_eth *eth, struct sk_buff *skb)
{

	u32 length = skb->len;
	ethTxMsg_t ethTxMsg;
	int error;

	if (skb->data == NULL) {
		printk("Tx a empty mbuf\n"); 	
		return 1;
	}

	if (length < 60) {
		length = 60;
	}
	//qdma_bm_dump_dscp();


  // start filling tx message
  /* GDMA1 */
    memset(&ethTxMsg, 0, sizeof(ethTxMsg_t));
    ethTxMsg.raw.fport = GDM_P_GDMA1;
    ethTxMsg.raw.channel = 0;
    ethTxMsg.raw.queue =0;
#ifdef TCSUPPORT_CPU_EN7580
		ethTxMsg.raw.nboq = 0;
		ethTxMsg.raw.mtr_g = 0x7f; /*not use any meter ratelimit*/
#endif
    dev_err(eth->dev, "tc3162_eth_send, qdma_bm_transmit_packet\n");
		dump_skb(skb);
		error = qdma_bm_transmit_packet(eth, skb, 0,ethTxMsg.msg[0],ethTxMsg.msg[1]) ;
//		qdma_bm_dump_dscp();

		if(error){
			kfree_skb(skb);
			printk("qdma transmit fail\n");
			return 0;
		}

	return 0;
}

static const struct phylink_mac_ops airoha_phylink_ops = {
//	.validate = phylink_generic_validate,
//	.mac_select_pcs = mtk_mac_select_pcs,
//	.mac_config = mtk_mac_config,
//	.mac_finish = mtk_mac_finish,
//	.mac_link_down = mtk_mac_link_down,
//	.mac_link_up = mtk_mac_link_up,
};

static netdev_tx_t airoha_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct airoha_mac *mac = netdev_priv(dev);
	struct airoha_eth *eth = mac->hw;
	dev_err(eth->dev, "SKB tranmit\n");
	//dump_skb(skb);
	tc3162_eth_send(eth, skb);
	return NETDEV_TX_OK;
}

static void airoha_uninit(struct net_device *dev)
{
}

static int airoha_stop(struct net_device *dev)
{
	return 0;
}

static int airoha_change_mtu(struct net_device *dev, int new_mtu)
{
	return 0;
}

static int airoha_xdp_xmit(struct net_device *dev, int num_frame,
			struct xdp_frame **frames, u32 flags)
{
	int nxmit = 0;

	return nxmit;
}

static int airoha_open(struct net_device *dev)
{
	return 0;
}


static const struct net_device_ops airoha_netdev_ops = {
	.ndo_uninit		= airoha_uninit,
	.ndo_open		= airoha_open,
	.ndo_stop		= airoha_stop,
	.ndo_start_xmit		= airoha_start_xmit,
//	.ndo_set_mac_address	= mtk_set_mac_address,
//	.ndo_validate_addr	= eth_validate_addr,
//	.ndo_eth_ioctl		= mtk_do_ioctl,
	.ndo_change_mtu		= airoha_change_mtu,
//	.ndo_tx_timeout		= mtk_tx_timeout,
//	.ndo_get_stats64        = mtk_get_stats64,
//	.ndo_fix_features	= mtk_fix_features,
//	.ndo_set_features	= mtk_set_features,
#ifdef CONFIG_NET_POLL_CONTROLLER
//	.ndo_poll_controller	= mtk_poll_controller,
#endif
//	.ndo_setup_tc		= mtk_eth_setup_tc,
//	.ndo_bpf		= mtk_xdp,
	.ndo_xdp_xmit		= airoha_xdp_xmit,
//	.ndo_select_queue	= mtk_select_queue,
};

static int airoha_add_mac(struct airoha_eth *eth, struct device_node *np)
{
	const __be32 *_id = of_get_property(np, "reg", NULL);
	phy_interface_t phy_mode;
	struct phylink *phylink;
	struct airoha_mac *mac;
	int id, err;
	int txqs = 1;


	if (!_id) {
		dev_err(eth->dev, "missing mac id\n");
		return -EINVAL;
	}

	id = be32_to_cpup(_id);
	dev_err(eth->dev, "airoha_add_mac: %d\n", id);

	if (eth->netdev[id]) {
		dev_err(eth->dev, "duplicate mac id found: %d\n", id);
		return -EINVAL;
	}


	eth->netdev[id] = alloc_etherdev_mqs(sizeof(*mac), txqs, 1);
	if (!eth->netdev[id]) {
		dev_err(eth->dev, "alloc_etherdev failed\n");
		return -ENOMEM;
	}
	mac = netdev_priv(eth->netdev[id]);
	eth->mac[id] = mac;
	mac->id = id;
	mac->hw = eth;
	mac->of_node = np;

	err = of_get_ethdev_address(mac->of_node, eth->netdev[id]);
	if (err == -EPROBE_DEFER)
		return err;

	if (err) {
		/* If the mac address is invalid, use random mac address */
		eth_hw_addr_random(eth->netdev[id]);
		dev_err(eth->dev, "generated random MAC address %pM\n",
			eth->netdev[id]->dev_addr);
	}

///	memset(mac->hwlro_ip, 0, sizeof(mac->hwlro_ip));
//	mac->hwlro_ip_cnt = 0;

// 	mac->hw_stats = devm_kzalloc(eth->dev,
// 				     sizeof(*mac->hw_stats),
// 				     GFP_KERNEL);
// 	if (!mac->hw_stats) {
// 		dev_err(eth->dev, "failed to allocate counter memory\n");
// 		err = -ENOMEM;
// 		goto free_netdev;
// 	}
// 	spin_lock_init(&mac->hw_stats->stats_lock);
// 	u64_stats_init(&mac->hw_stats->syncp);


	/* phylink create */
	err = of_get_phy_mode(np, &phy_mode);
	if (err) {
		dev_err(eth->dev, "incorrect phy-mode\n");
		goto free_netdev;
	}

	/* mac config is not set */
	mac->interface = PHY_INTERFACE_MODE_NA;
	mac->speed = SPEED_UNKNOWN;

	mac->phylink_config.dev = &eth->netdev[id]->dev;
	mac->phylink_config.type = PHYLINK_NETDEV;
	mac->phylink_config.mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
		MAC_10 | MAC_100 | MAC_1000 | MAC_2500FD;

	dev_err(eth->dev, "phylink\n");

//	phy_interface_set_rgmii(mac->phylink_config.supported_interfaces);

// 	phylink = phylink_create(&mac->phylink_config,
// 				 of_fwnode_handle(mac->of_node),
// 				 phy_mode, &airoha_phylink_ops);
// 	if (IS_ERR(phylink)) {
// 		err = PTR_ERR(phylink);
// 		goto free_netdev;
// 	}
// 
// 	mac->phylink = phylink;

	SET_NETDEV_DEV(eth->netdev[id], eth->dev);
	dev_err(eth->dev, "SET_NETDEV_DEV()\n");
	eth->netdev[id]->watchdog_timeo = 5 * HZ;
	dev_err(eth->dev, "watchdog_timeo\n");
	eth->netdev[id]->netdev_ops = &airoha_netdev_ops;
	dev_err(eth->dev, "netdev_ops\n");
	eth->netdev[id]->base_addr = (unsigned long)eth->base;
	dev_err(eth->dev, "base_addr(%x)\n",eth->netdev[id]->base_addr);

	eth->netdev[id]->hw_features = eth->soc->hw_features;
	if (eth->hwlro)
		eth->netdev[id]->hw_features |= NETIF_F_LRO;
	dev_err(eth->dev, "netdev\n");

	eth->netdev[id]->vlan_features = eth->soc->hw_features &
		~NETIF_F_HW_VLAN_CTAG_TX;
	eth->netdev[id]->features |= eth->soc->hw_features;
//	eth->netdev[id]->ethtool_ops = &mtk_ethtool_ops;

	eth->netdev[id]->irq = eth->irq[0];
	eth->netdev[id]->dev.of_node = np;

	eth->netdev[id]->max_mtu = 1500;

// 	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
// 		mac->device_notifier.notifier_call = mtk_device_event;
// 		register_netdevice_notifier(&mac->device_notifier);
// 	}

	return 0;

free_netdev:
	free_netdev(eth->netdev[id]);
	return err;
}

void macResetSwMAC(struct airoha_eth *eth)
{
	u32 reg;
#if defined(TCSUPPORT_CPU_EN7580)
	//SPI_NAND_FLASH_RTN_T status;
#endif

	dev_err(eth->dev, "macResetSwMAC: %x \n", eth->rst_2);
	/* reset ethernet phy, ethernet switch, frame engine */
	reg = ioread32(eth->rst_2);
	reg |= (QDMA1_RST | QDMA2_RST | FE_RST);
	iowrite32(reg, eth->rst_2);

	msleep(20);

	/* de-assert reset ethernet phy, ethernet switch, frame engine */
	reg = ioread32(eth->rst_2);
	reg &= ~(QDMA1_RST | QDMA2_RST | FE_RST);
	iowrite32(reg, eth->rst_2);

	/* add delay time to prevent switch reg I/O hang */
    msleep(20);

	/* EN7580: read flash info to register, for prom init */
#if defined(TCSUPPORT_CPU_EN7580)
	//VPint(QDMA_WAN_DBG_MEM_XS_DATA_HI) = READ_FLASH_BYTE(FLASH_QDMA_INIT, &status);
#endif

}


static int airoha_probe(struct platform_device *pdev)
{
//	struct resource *res = NULL, *res_sram;
	struct device_node *mac_np;
	struct airoha_eth *eth;
	int err, i, ret;

	dev_err(&pdev->dev, "airoha_eth_probe\n");
	eth = devm_kzalloc(&pdev->dev, sizeof(*eth), GFP_KERNEL);
	if (!eth)
		return -ENOMEM;

	eth->soc = of_device_get_match_data(&pdev->dev);
	
	/* FE Base */
	eth->dev = &pdev->dev;
	eth->dma_dev = &pdev->dev;
	eth->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(eth->base))
		return PTR_ERR(eth->base);

	/* QDMA Base */
	eth->qdma_base[0] = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(eth->qdma_base[0]))
		return PTR_ERR(eth->qdma_base[0]);

	eth->qdma_base[1] = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(eth->qdma_base[1]))
		return PTR_ERR(eth->qdma_base[1]);

	eth->rst_2 = devm_platform_ioremap_resource(pdev, 3);
	if (IS_ERR(eth->rst_2))
		return PTR_ERR(eth->rst_2);
	
	eth->esw = eth->rst_2 + 0x8000;

	for_each_child_of_node(pdev->dev.of_node, mac_np) {
		if (!of_device_is_compatible(mac_np,
					     "mediatek,eth-mac"))
			continue;

		if (!of_device_is_available(mac_np))
			continue;
		
		macResetSwMAC(eth);

		err = airoha_add_mac(eth, mac_np);
		if (err) {
			of_node_put(mac_np);
			goto err_deinit_hw;
		}
	}
	
	/* IRQ */
	for (i = 0; i < 10; i++) {
		dev_err(&pdev->dev, "IRQ%d resource loop\n", i);
		eth->irq[i] = platform_get_irq(pdev, i);
		if (eth->irq[i] < 0) {
			dev_err(&pdev->dev, "no IRQ%d resource found\n", i);
			err = -ENXIO;
			goto err_irq_exit;
		}
	}

	/* QDMA INIT */
	memset(gpQdmaPriv, 0, sizeof(QDMA_Private_T));

	gpQdmaPriv->dbgLevel = DBG_ERR ;
	/* Initial for design and verification function */
	gpQdmaPriv->txDscpNum = TX0_DSCP_NUM;
	gpQdmaPriv->rxDscpNum = RX0_DSCP_NUM;
	gpQdmaPriv->hwFwdDscpNum = HWFWD_DSCP_NUM;
	gpQdmaPriv->irqDepth = IRQ_DEPTH;
	gpQdmaPriv->hwPktSize = MAX_PKT_LENS;
	gpQdmaPriv->csrBaseAddr = eth->qdma_base[0];
	dev_err(&pdev->dev, "gpQdmaPriv->csrBaseAddr: %p\n", gpQdmaPriv->csrBaseAddr);
	
	if((ret = qdma_bm_dscp_init(eth)) != 0) 
	{
		dev_err(&pdev->dev, "QDMA DSCP initialization failed.\n") ;
		return ret ;
	}
	/***************************************************
	* QDMA device initialization                       *
	****************************************************/
	dev_err(&pdev->dev, "QDMA device initialization\n") ;
	if((ret = qdma_dev_init(eth)) != 0) {
		dev_err(&pdev->dev, "QDMA hardware device initialization failed.\n") ;
		return ret ;
	}
	qdmaEnableTxDma(gpQdmaPriv->csrBaseAddr);
	qdmaEnableRxDma(gpQdmaPriv->csrBaseAddr);

//	eth->msg_enable = netif_msg_init(airoha_msg_level, AIROHA_DEFAULT_MSG_ENABLE);
//	INIT_WORK(&eth->pending_work, airoha_pending_work);

	dev_err(&pdev->dev, "macGetMacAddr\n") ;
	macGetMacAddr(eth, mac_p, eth->netdev[0]->dev_addr);
	macDrvRegInit(eth, mac_p);

	for (i = 0; i < /*AIROHA_MAX_DEVS*/ 1; i++) {
		if (!eth->netdev[i])
			continue;

		err = register_netdev(eth->netdev[i]);
		if (err) {
			dev_err(eth->dev, "error bringing up device\n");
			goto err_deinit_hw;
		} else
			netif_info(eth, probe, eth->netdev[i],
				   "airoha frame engine at 0x%08lx, irq %d\n",
				   eth->netdev[i]->base_addr, eth->irq[0]);
	}

	
	dev_err(&pdev->dev, "Probe ok\n");


  return 0;
err_irq_exit:
err_deinit_hw:
  return err;
}


static int airoha_remove(struct platform_device *pdev)
{
	struct airoha_eth *eth = platform_get_drvdata(pdev);
//	struct airoha_mac *mac;
	int i;

  dev_err(&pdev->dev, "airoha_remove\n");
	/* stop all devices to make sure that dma is properly shut down */
	for (i = 0; i < AIROHA_MAX_DEVS; i++) {
		if (!eth->netdev[i])
			continue;
//		airoha_stop(eth->netdev[i]);
//		mac = netdev_priv(eth->netdev[i]);
//		phylink_disconnect_phy(mac->phylink);
	}

//	mtk_wed_exit();
//	mtk_hw_deinit(eth);

	netif_napi_del(&eth->tx_napi);
	netif_napi_del(&eth->rx_napi);
//	mtk_cleanup(eth);
//	mtk_mdio_cleanup(eth);

	return 0;
}

const struct of_device_id of_airoha_match[] = {
	{ .compatible = "airoha,en7523-eth", .data = &en7523_data },
//	{ .compatible = "airoha,an7581-eth", .data = &an7581_data },
	{},
};
MODULE_DEVICE_TABLE(of, of_airoha_match);

static struct platform_driver airoha_driver = {
	.probe = airoha_probe,
	.remove = airoha_remove,
	.driver = {
		.name = "airoha_soc_eth",
		.of_match_table = of_airoha_match,
	},
};

module_platform_driver(airoha_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Benjamin Larsson <benjamin.larsson@genexis.eu>");
MODULE_DESCRIPTION("Ethernet driver for Airoha SoC");
