/**
 * <b>File:</b> stm32x_ethernetif.c
 *
 * <b>Project:</b> STM32x Eclipse demo
 *
 * <b>Description:</b> This module is used to implement a network interface bridge between
 *                     the STM32x MAC peripheral and lwIP stack.
 *
 *
 * <b>Created:</b> 27/06/2009
 *
 * <dl>
 * <dt><b>Author</b>:</dt>
 * <dd>Stefano Oliveri</dd>
 * <dt><b>E-mail:</b></dt>
 * <dd>software@stf12.net</dd>
 * </dl>
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include <netif/etharp.h>

// ST Library include
#include "stm32f10x.h"
#include "misc.h"
#include <stm32_eth.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

// FreeRTOS include
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

// Standard library include
#include <stdio.h>
#include <string.h>

#define netifMTU								( 1500 )
#define netifINTERFACE_TASK_STACK_SIZE			( 1600 )
#define netifINTERFACE_TASK_PRIORITY			( configMAX_PRIORITIES - 6 )
#define IFNAME0 'e'
#define IFNAME1 'n'
#define netifINIT_WAIT							( 100 / portTICK_RATE_MS )

/* The time to block waiting for input. */
#define emacBLOCK_TIME_WAITING_FOR_INPUT		( ( portTickType ) 100 )


/* Hardware specifics. */
#define RCC_MAC_CLOCK							( 1UL << 14UL )
#define RCC_MAC_TX_CLOCK						( 1UL << 15UL )
#define RCC_MAC_RX_CLOCK						( 1UL << 16UL )
#define PHY_ADDRESS								( 1 )
#define MODE_MII								( 1UL << 23UL )
#define REMAP_MAC_IO							( 1UL << 21UL )

/* The number of descriptors to chain together for use by the Rx DMA. */
#define NUM_RX_DESCRIPTORS						4

/* The total number of buffers to be available.  At most (?) there should be
 one available for each Rx descriptor, one for current use, and one that is
 in the process of being transmitted. */
#define NUM_BUFFERS								( NUM_RX_DESCRIPTORS + 2 )

/* Each buffer is sized to fit an entire Ethernet packet.  This is for
 simplicity and speed, but could waste RAM. */
#define MAX_PACKET_SIZE							1520

/* The field in the descriptor that is unused by this configuration is used to
 hold the send count.  This is just #defined to a meaningful name. */
#define SendCount Buffer2NextDescAddr

/* If no buffers are available, then wait this long before looking again.... */
#define netifBUFFER_WAIT_DELAY					( 100 / portTICK_RATE_MS )
/* ...and don't look more than this many times. */
#define netifBUFFER_WAIT_ATTEMPTS				( 9 )
/* Let the DMA know that a new descriptor has been made available to it. */
#define prvRxDescriptorAvailable()				ETH->DMARPDR = 0

/* Allocate the Rx descriptors used by the DMA. */
static ETH_DMADESCTypeDef xRxDescriptors[NUM_RX_DESCRIPTORS] __attribute__((aligned(4)));

/* Allocate the descriptor used for transmitting.  It might be that better
 performance could be achieved by having more than one Tx descriptor, but
 in this simple case only one is used. */
static volatile ETH_DMADESCTypeDef xTxDescriptor __attribute__((aligned(4)));

/* Buffers used for receiving and transmitting data. */
static unsigned char ucMACBuffers[NUM_BUFFERS][MAX_PACKET_SIZE] __attribute__((aligned(4)));

/* Each ucBufferInUse index corresponds to a position in the same index in the
 ucMACBuffers array.  If the index contains a 1 then the buffer within
 ucMACBuffers is in use, if it contains a 0 then the buffer is free. */
static unsigned char ucBufferInUse[NUM_BUFFERS] = { 0 };

/* Index to the Rx descriptor to inspect next when looking for a received
 packet. */
static unsigned long s_ulNextDescriptor;

/* The lwip_buffer for Rx packet is not a fixed array, but instead gets pointed to the buffers
 allocated within this file. */
static unsigned char * s_lwip_in_buf;

/* The lwip_buffer for Tx packet is not a fixed array, but instead gets pointed to the buffers
 allocated within this file. */
static unsigned char * s_lwip_out_buf;

/* Flag to indicate transmit machine is ready to fire off an outgoing packet */
static char s_tx_ready;

/* The semaphore used by the ISR to wake the lwIP task. */
static xSemaphoreHandle s_xSemaphore = NULL;

struct ethernetif {
	struct eth_addr *ethaddr;
/* Add whatever per-interface state that is needed here. */
};

/* Forward declarations. */
static void ethernetif_input(void *pParams);

/**
 * Configure the IO for Ethernet use.
 */
static void prvSetupEthGPIO(void);

/**
 * Return a pointer to an unused buffer, marking the returned buffer as now
 * in use.
 */
static unsigned char *prvGetNextBuffer(void);

/**
 * Send usDataLen bytes from s_lwip_buf to the MAC for transmission.
 */
unsigned char vSendMACData(unsigned short usDataLen);

/**
 * Look for new received data.  If any is found then set s_lwip_buf to point to the
 * data and return the length of the data.  If no data is found then 0 is
 * returned, and s_lwip_buf is left pointing to a spare data buffer.
 */
unsigned short usGetMACRxData(void);

/**
 * Initialize all IO and peripherals required for Ethernet communication.
 *
 * @return pdSUCCESS if success, pdFAIL to signal some error.
 */
portBASE_TYPE xEthInitialise(void);

/**
 * Return a buffer to the pool of free buffers.
 */
void vReturnBuffer(unsigned char *pucBuffer);


/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif) {
	/* set MAC hardware address length */
	netif->hwaddr_len = ETHARP_HWADDR_LEN;
	/* set MAC hardware address */
	netif->hwaddr[0] = configMAC_ADDR0;
	netif->hwaddr[1] = configMAC_ADDR1;
	netif->hwaddr[2] = configMAC_ADDR2;
	netif->hwaddr[3] = configMAC_ADDR3;
	netif->hwaddr[4] = configMAC_ADDR4;
	netif->hwaddr[5] = configMAC_ADDR5;
	/* maximum transfer unit */
	netif->mtu = netifMTU;
	// device capabilities.
	// don't set NETIF_FLAG_ETHARP if this device is not an ethernet one
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP| NETIF_FLAG_LINK_UP;
	/* Do whatever else is needed to initialize interface. */
	if (s_xSemaphore == NULL) {
		vSemaphoreCreateBinary( s_xSemaphore );
		xSemaphoreTake( s_xSemaphore, 0);
	}

	/* Initialize the MAC. */
	while (xEthInitialise() != pdPASS) {
		vTaskDelay(netifINIT_WAIT);
	}

	netif->flags |= NETIF_FLAG_LINK_UP;

	/* Create the task that handles the EMAC. */
	xTaskCreate(ethernetif_input, (signed portCHAR *) "ETH_INT",
			netifINTERFACE_TASK_STACK_SIZE, (void *) netif,
			netifINTERFACE_TASK_PRIORITY, NULL);
}



/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p) {
	struct pbuf *q;
	int i;
	u32_t l = 0;
	err_t res = ERR_OK;

#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

	for (q = p; q != NULL; q = q->next) {
		/* Send the data from the pbuf to the interface, one pbuf at a
		 time. The size of the data in each pbuf is kept in the ->len
		 variable. */
		memcpy(&s_lwip_out_buf[l], (u8_t*) q->payload, q->len);
		l += q->len;
	}
	if ( !vSendMACData(l) )
		res = ERR_BUF;

#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

	LINK_STATS_INC(link.xmit);

	return res;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *low_level_input(struct netif *netif) {
	struct pbuf *p, *q;
	u16_t len, l;

	l = 0;
	p = NULL;

	/* Obtain the size of the packet and put it into the "len"
	 variable. */
	len = usGetMACRxData();

	if (len) {
#if ETH_PAD_SIZE
		len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

		/* We allocate a pbuf chain of pbufs from the pool. */
		p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

		if (p != NULL) {

#if ETH_PAD_SIZE
			pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

			/* We iterate over the pbuf chain until we have read the entire
			 * packet into the pbuf. */
			for (q = p; q != NULL; q = q->next) {
				/* Read enough bytes to fill this pbuf in the chain. The
				 * available data in the pbuf is given by the q->len
				 * variable. */
				memcpy((u8_t*) q->payload, &s_lwip_in_buf[l], q->len);
				l = l + q->len;
			}

#if ETH_PAD_SIZE
			pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

			LINK_STATS_INC(link.recv);

		} else {

			LINK_STATS_INC(link.memerr); LINK_STATS_INC(link.drop);

		} /* End else */
	} /* End if */

	vReturnBuffer(s_lwip_in_buf);
	s_lwip_in_buf = 0;

	return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface.Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */

static void ethernetif_input(void *pParams) {
	struct netif *netif;
	struct ethernetif *ethernetif;
	struct eth_hdr *ethhdr;
	struct pbuf *p;

	netif = (struct netif*) pParams;
	ethernetif = netif->state;

	for (;;) {
		do {

			/* move received packet into a new pbuf */
			p = low_level_input(netif);

			if (p == NULL) {
				/* No packet could be read.  Wait a for an interrupt to tell us
				 there is more data available. */
				xSemaphoreTake( s_xSemaphore, emacBLOCK_TIME_WAITING_FOR_INPUT );
			}

		} while (p == NULL);

		/* points to packet payload, which starts with an Ethernet header */
		ethhdr = p->payload;

		switch (htons(ethhdr->type)) {
		/* IP or ARP packet? */

		case ETHTYPE_IP:
		case ETHTYPE_ARP:

			/* full packet send to tcpip_thread to process */
			if (netif->input(p, netif) != ERR_OK) {
				LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
				pbuf_free(p);
				p = NULL;
			}
			break;

		default:
			pbuf_free(p);
			p = NULL;
			break;
		}
	}
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif) {
	struct ethernetif *ethernetif;

	LWIP_ASSERT("netif != NULL", (netif != NULL));

	ethernetif = mem_malloc(sizeof(struct ethernetif));

	if (ethernetif == NULL) {
		LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
		return ERR_MEM;
	}

#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

	/*
	 * Initialize the snmp variables and counters inside the struct netif.
	 * The last argument should be replaced with your link speed, in units
	 * of bits per second.
	 */
	NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, 100);

	netif->state = ethernetif;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;
	/* We directly use etharp_output() here to save a function call.
	 * You can instead declare your own function an call etharp_output()
	 * from it if you have to do some checks before sending (e.g. if link
	 * is available...)
	 */
	netif->output = etharp_output;
	netif->linkoutput = low_level_output;

	ethernetif->ethaddr = (struct eth_addr *) &(netif->hwaddr[0]);

	low_level_init(netif);

	return ERR_OK;
}

/**
 * Initialize all IO and peripherals required for Ethernet communication.
 *
 * @return pdSUCCESS if success, pdFAIL to signal some error.
 */
portBASE_TYPE xEthInitialise(void) {
	static ETH_InitTypeDef xEthInit; /* Static so as not to take up too much stack space. */
	NVIC_InitTypeDef xNVICInit;
	const unsigned char ucMACAddress[] = { configMAC_ADDR0, configMAC_ADDR1, configMAC_ADDR2,
			configMAC_ADDR3, configMAC_ADDR4, configMAC_ADDR5 };
	portBASE_TYPE xReturn;
	unsigned long ul;

	/* Start with things in a safe known state. */
	ETH_DeInit();
	for (ul = 0; ul < NUM_RX_DESCRIPTORS; ul++) {
		ETH_DMARxDescReceiveITConfig(&(xRxDescriptors[ul]), DISABLE);
	}
	/* Enable ETHERNET clock  */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC | RCC_AHBPeriph_ETH_MAC_Tx |
					RCC_AHBPeriph_ETH_MAC_Rx, ENABLE);

	/* Use MII mode. */
	GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_RMII);

	/* Enable GPIOs clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |	RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
					RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE| RCC_APB2Periph_AFIO, ENABLE);

	/* Get HSE clock = 25MHz on PA8 pin(MCO) */
	/* set PLL3 clock output to 50MHz (25MHz /5 *10 =50MHz) */
	RCC_PLL3Config(RCC_PLL3Mul_10);
	/* Enable PLL3 */
	RCC_PLL3Cmd(ENABLE);
	/* Wait till PLL3 is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET)
	{}

	/* Get clock PLL3 clock on PA8 pin */
	RCC_MCOConfig(RCC_MCO_PLL3CLK);

	/* Configure all the GPIO as required for MAC/PHY interfacing. */
	prvSetupEthGPIO();

	/* Reset the peripheral. */
	ETH_SoftwareReset();
	while (ETH_GetSoftwareResetStatus() == SET);

	/* Set the MAC address. */
	ETH_MACAddressConfig(ETH_MAC_Address0, (unsigned char *) ucMACAddress);



	/* Initialise using the whopping big structure.  Code space could be saved
	 by making this a const struct, however that would mean changes to the
	 structure within the library header files could break the code, so for now
	 just set everything manually at run time. */
	xEthInit.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
	xEthInit.ETH_Watchdog = ETH_Watchdog_Disable;
	xEthInit.ETH_Jabber = ETH_Jabber_Disable;
	//xEthInit.ETH_JumboFrame = ETH_JumboFrame_Disable;
	xEthInit.ETH_InterFrameGap = ETH_InterFrameGap_96Bit;
	xEthInit.ETH_CarrierSense = ETH_CarrierSense_Enable;
	xEthInit.ETH_Speed = ETH_Speed_10M;
	xEthInit.ETH_ReceiveOwn = ETH_ReceiveOwn_Disable;
	xEthInit.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
	xEthInit.ETH_Mode = ETH_Mode_HalfDuplex;
	xEthInit.ETH_ChecksumOffload = ETH_ChecksumOffload_Disable;
	xEthInit.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
	xEthInit.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
	xEthInit.ETH_BackOffLimit = ETH_BackOffLimit_10;
	xEthInit.ETH_DeferralCheck = ETH_DeferralCheck_Disable;
	xEthInit.ETH_ReceiveAll = ETH_ReceiveAll_Enable;
	xEthInit.ETH_SourceAddrFilter = ETH_SourceAddrFilter_Disable;
	xEthInit.ETH_PassControlFrames
			= ETH_PassControlFrames_ForwardPassedAddrFilter;
	xEthInit.ETH_BroadcastFramesReception
			= ETH_BroadcastFramesReception_Disable;
	xEthInit.ETH_DestinationAddrFilter = ETH_DestinationAddrFilter_Normal;
	xEthInit.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
	xEthInit.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
	xEthInit.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
	xEthInit.ETH_HashTableHigh = 0x0;
	xEthInit.ETH_HashTableLow = 0x0;
	xEthInit.ETH_PauseTime = 0x0;
	xEthInit.ETH_ZeroQuantaPause = ETH_ZeroQuantaPause_Disable;
	xEthInit.ETH_PauseLowThreshold = ETH_PauseLowThreshold_Minus4;
	xEthInit.ETH_UnicastPauseFrameDetect = ETH_UnicastPauseFrameDetect_Disable;
	xEthInit.ETH_ReceiveFlowControl = ETH_ReceiveFlowControl_Disable;
	xEthInit.ETH_TransmitFlowControl = ETH_TransmitFlowControl_Disable;
	xEthInit.ETH_VLANTagComparison = ETH_VLANTagComparison_16Bit;
	xEthInit.ETH_VLANTagIdentifier = 0x0;
	xEthInit.ETH_DropTCPIPChecksumErrorFrame
			= ETH_DropTCPIPChecksumErrorFrame_Disable;
	xEthInit.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
	xEthInit.ETH_FlushReceivedFrame = ETH_FlushReceivedFrame_Disable;
	xEthInit.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;
	xEthInit.ETH_TransmitThresholdControl
			= ETH_TransmitThresholdControl_64Bytes;
	xEthInit.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;
	xEthInit.ETH_ForwardUndersizedGoodFrames
			= ETH_ForwardUndersizedGoodFrames_Disable;
	xEthInit.ETH_ReceiveThresholdControl = ETH_ReceiveThresholdControl_64Bytes;
	xEthInit.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Disable;
	xEthInit.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
	xEthInit.ETH_FixedBurst = ETH_FixedBurst_Disable;
	xEthInit.ETH_RxDMABurstLength = ETH_RxDMABurstLength_1Beat;
	xEthInit.ETH_TxDMABurstLength = ETH_TxDMABurstLength_1Beat;
	xEthInit.ETH_DescriptorSkipLength = 0x0;
	xEthInit.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_1_1;

	xReturn = ETH_Init(&xEthInit, PHY_ADDRESS);

	/* Check a link was established. */
	if (xReturn != pdFAIL) {
		/* Rx and Tx interrupts are used. */
		ETH_DMAITConfig(ETH_DMA_IT_NIS | ETH_DMA_IT_R | ETH_DMA_IT_T, ENABLE);

		/* Only a single Tx descriptor is used.  For now it is set to use an Rx
		 buffer, but will get updated to point to where ever s_lwip_buf is
		 pointing prior to its use. */
		ETH_DMATxDescChainInit((void *) &xTxDescriptor, (void *) ucMACBuffers, 1);
		ETH_DMARxDescChainInit(xRxDescriptors, (void *) ucMACBuffers, NUM_RX_DESCRIPTORS);
		for (ul = 0; ul < NUM_RX_DESCRIPTORS; ul++) {
			/* Ensure received data generates an interrupt. */
			ETH_DMARxDescReceiveITConfig(&(xRxDescriptors[ul]), ENABLE);

			/* Fix up the addresses used by the descriptors.
			 The way ETH_DMARxDescChainInit() is not compatible with the buffer
			 declarations in this file. */
			xRxDescriptors[ul].Buffer1Addr = (unsigned long) &(ucMACBuffers[ul][0]);

			/* Mark the buffer used by this descriptor as in use. */
			ucBufferInUse[ul] = pdTRUE;
		}

		/* When receiving data, start at the first descriptor. */
		s_ulNextDescriptor = 0;

		/* Initialize s_lwip_buf to ensure it points somewhere valid. */
		s_lwip_in_buf = 0;
		s_lwip_out_buf = prvGetNextBuffer();

		/* Mark the tx machine as ready */
		s_tx_ready = 1;

		/* SendCount must be initialized to 2 to ensure the Tx descriptor looks
		 as if its available (as if it has already been sent twice. */
		//xTxDescriptor.SendCount = 2;

//#ifdef CHECKSUM_BY_HARDWARE
		/* Enable the checksum insertion for the Tx frames */
		ETH_DMATxDescChecksumInsertionConfig(&xTxDescriptor, ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
//#endif

		/* Switch on the interrupts in the NVIC. */
		xNVICInit.NVIC_IRQChannel = ETH_IRQn;
		xNVICInit.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
		xNVICInit.NVIC_IRQChannelSubPriority = 0;
		xNVICInit.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&xNVICInit);

		/* Buffers and descriptors are all set up, now enable the MAC. */
		ETH_Start();

		/* Let the DMA know there are Rx descriptors available. */
		prvRxDescriptorAvailable();
		/*Disable interrupts*/
		//NVIC_DisableIRQ(ETH_IRQn);
	}

	return xReturn;
}

/**
 * Configure the IO for Ethernet use.
 */
static void prvSetupEthGPIO(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ETHERNET pins configuration */
	/* AF Output Push Pull:
	- ETH_MII_MDIO / ETH_RMII_MDIO: PA2
	- ETH_MII_MDC / ETH_RMII_MDC: PC1
	- ETH_MII_TXD2: PC2
	- ETH_MII_TX_EN / ETH_RMII_TX_EN: PB11
	- ETH_MII_TXD0 / ETH_RMII_TXD0: PB12
	- ETH_MII_TXD1 / ETH_RMII_TXD1: PB13
	- ETH_MII_PPS_OUT / ETH_RMII_PPS_OUT: PB5
	- ETH_MII_TXD3: PB8 */

	/* Configure PA2 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PC1, PC2 and PC3 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PB5, PB8, PB11, PB12 and PB13 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 |
	GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/**************************************************************/
	/*               For Remapped Ethernet pins                   */
	/*************************************************************/
	/* Input (Reset Value):
	- ETH_MII_CRS CRS: PA0
	- ETH_MII_RX_CLK / ETH_RMII_REF_CLK: PA1
	- ETH_MII_COL: PA3
	- ETH_MII_RX_DV / ETH_RMII_CRS_DV: PD8
	- ETH_MII_TX_CLK: PC3
	- ETH_MII_RXD0 / ETH_RMII_RXD0: PD9
	- ETH_MII_RXD1 / ETH_RMII_RXD1: PD10
	- ETH_MII_RXD2: PD11
	- ETH_MII_RXD3: PD12
	- ETH_MII_RX_ER: PB10 */

	/* ETHERNET pins remapp in STM3210C-EVAL board: RX_DV and RxD[3:0] */
	GPIO_PinRemapConfig(GPIO_Remap_ETH, DISABLE);

	/* Configure PA0, PA1 and PA3 as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PB10 as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PC3 as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PD8, PD9, PD10, PD11 and PD12 as input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure); /**/

	/* MCO pin configuration------------------------------------------------- */
	/* Configure MCO (PA8) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static unsigned char *prvGetNextBuffer(void) {
	portBASE_TYPE x;
	unsigned char *ucReturn = NULL;

	while (ucReturn == NULL) {
		/* Look through the buffers to find one that is not in use by
		 anything else. */
		vTaskSuspendAll();
		for (x = 0; x < NUM_BUFFERS; x++) {
			if (ucBufferInUse[x] == pdFALSE) {
				ucBufferInUse[x] = pdTRUE;
				ucReturn = &(ucMACBuffers[x][0]);
				break;
			}
		}
		xTaskResumeAll();

		/* Was a buffer found? */
		if (ucReturn == NULL) {
			/* Wait then look again. */
			vTaskDelay(netifBUFFER_WAIT_DELAY);
		}
	}
	return ucReturn;
}

void vMAC_ISR(void) {
	unsigned long ulStatus;
	long xHigherPriorityTaskWoken = pdFALSE;

	/* What caused the interrupt? */
	ulStatus = ETH->DMASR;

	/* Clear everything before leaving. */
	ETH->DMASR = ulStatus;

	if (ulStatus & ETH_DMA_IT_R) {
		/* Data was received.  Ensure the uIP task is not blocked as data has
		 arrived. */
		xSemaphoreGiveFromISR( s_xSemaphore, &xHigherPriorityTaskWoken );
	}

	if (ulStatus & ETH_DMA_IT_T) {
		/* Data was transmitted, ready to transmit again */
		s_tx_ready = 1;
		/* The Tx buffer is no longer required. */
		vReturnBuffer((unsigned char *) xTxDescriptor.Buffer1Addr);
	}

	/* If xSemaphoreGiveFromISR() unblocked a task, and the unblocked task has
	 a higher priority than the currently executing task, then
	 xHigherPriorityTaskWoken will have been set to pdTRUE and this ISR should
	 return directly to the higher priority unblocked task. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void ETH_IRQHandler()
{
	vMAC_ISR();
}

void DMA2_Channel5_IRQHandler()
{
	vMAC_ISR();
}


void vReturnBuffer(unsigned char *pucBuffer) {
	unsigned long ul;

	/* Mark a buffer as free for use. */
	for (ul = 0; ul < NUM_BUFFERS; ul++) {
		if (ucMACBuffers[ul] == pucBuffer) {
			ucBufferInUse[ul] = pdFALSE;
			break;
		}
	}
}

unsigned char vSendMACData(unsigned short usDataLen) {
	unsigned long ulAttempts = 0UL;
	unsigned char res = 1;

	/* Check to see if the Tx descriptor is free. */
	while (!s_tx_ready || (xTxDescriptor.Status & ETH_DMATxDesc_OWN) == ETH_DMATxDesc_OWN) {
		/* Wait for the Tx descriptor to become available. */
		vTaskDelay(netifBUFFER_WAIT_DELAY);

		ulAttempts++;
		if (ulAttempts > netifBUFFER_WAIT_ATTEMPTS) {
			/* Something has gone wrong as the Tx descriptor is still in use.
			 Clear it down manually, the data it was sending will probably be
			 lost. */
			xTxDescriptor.Status &= ~ETH_DMATxDesc_OWN;
			vReturnBuffer((unsigned char *) xTxDescriptor.Buffer1Addr);
			res = 0;
			break;
		}
	}

	/* Setup the Tx descriptor for transmission. */
	s_tx_ready = 0;
	xTxDescriptor.Buffer1Addr = (unsigned long) s_lwip_out_buf;
	xTxDescriptor.ControlBufferSize = (unsigned long) usDataLen;
	xTxDescriptor.Status = ETH_DMATxDesc_OWN | ETH_DMATxDesc_LS
			| ETH_DMATxDesc_FS | ETH_DMATxDesc_TER | ETH_DMATxDesc_TCH
			| ETH_DMATxDesc_IC;
	ETH->DMASR = ETH_DMASR_TBUS;
	ETH->DMATPDR = 0;

	/* s_lwip_out_buf is being sent by the Tx descriptor.  Allocate a new buffer. */
	//TODO: It causes problems under load. It  seems it is not thread safe.
	s_lwip_out_buf = prvGetNextBuffer();
	vTaskDelay(10);
	return res;
}

unsigned short usGetMACRxData(void) {
	unsigned short usReturn;

	if ((xRxDescriptors[s_ulNextDescriptor].Status & ETH_DMARxDesc_ES) != 0) {
		/* Error in Rx.  Discard the frame and give it back to the DMA. */
		xRxDescriptors[s_ulNextDescriptor].Status = ETH_DMARxDesc_OWN;
		prvRxDescriptorAvailable();

		/* No data to return. */
		usReturn = 0UL;

		/* Start from the next descriptor the next time this function is called. */
		s_ulNextDescriptor++;
		if (s_ulNextDescriptor >= NUM_RX_DESCRIPTORS) {
			s_ulNextDescriptor = 0UL;
		}
	} else if ((xRxDescriptors[s_ulNextDescriptor].Status & ETH_DMARxDesc_OWN) == 0) {
		/* Mark the current buffer as free as s_lwip_in_buf is going to be set to
		 the buffer that contains the received data. */
		if (s_lwip_in_buf)
			vReturnBuffer(s_lwip_in_buf);

		/* Get the received data length	from the top 2 bytes of the Status
		 word and the data itself. */
		usReturn = (unsigned short) ((xRxDescriptors[s_ulNextDescriptor].Status & ETH_DMARxDesc_FL) >> 16UL);
		s_lwip_in_buf = (unsigned char *) (xRxDescriptors[s_ulNextDescriptor].Buffer1Addr);

		/* Allocate a new buffer to the descriptor. */
		xRxDescriptors[s_ulNextDescriptor].Buffer1Addr = (unsigned long) prvGetNextBuffer();

		/* Give the descriptor back to the DMA. */
		xRxDescriptors[s_ulNextDescriptor].Status = ETH_DMARxDesc_OWN;
		prvRxDescriptorAvailable();

		/* Start from the next descriptor the next time this function is called. */
		s_ulNextDescriptor++;
		if (s_ulNextDescriptor >= NUM_RX_DESCRIPTORS) {
			s_ulNextDescriptor = 0UL;
		}
	} else {
		/* No received data at all. */
		usReturn = 0UL;
	}

	return usReturn;
}

