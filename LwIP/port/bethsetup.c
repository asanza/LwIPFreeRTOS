/*
 * bethsetup.c
 *
 *  Created on: 26.08.2012
 *      Author: diego
 */

#include <stm32_eth.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <misc.h>

#include <FreeRTOSConfig.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "beth.h"

/// public interface


/// internal interface
static void prvSetupEthGPIO(void);
/* The number of descriptors to chain together for use by the Rx DMA. */
#define NUM_RX_DESCRIPTORS		1
#define NUM_TX_DESCRIPTORS		1
// Number of buffers
#define NUM_RX_BUFFERS			2
#define NUM_TX_BUFFERS			1
/// ETH frame lenght
#define MAX_PACKET_SIZE			1520
/* ...and don't look more than this many times. */
#define netifBUFFER_WAIT_ATTEMPTS				( 9 )

/* Allocate the Rx and descriptors used by the DMA. */
static ETH_DMADESCTypeDef xRxDescriptors[NUM_RX_DESCRIPTORS] __attribute__((aligned(4)));
static ETH_DMADESCTypeDef xTxDescriptors[NUM_TX_DESCRIPTORS] __attribute__((aligned(4)));
/* Allocate separated buffers for receive and transmit data*/
static unsigned char rxMACBuffers[NUM_RX_BUFFERS][MAX_PACKET_SIZE] __attribute__((aligned(4)));
static unsigned char txMACBuffers[NUM_TX_BUFFERS][MAX_PACKET_SIZE] __attribute__((aligned(4)));
/* Each ucBufferInUse index corresponds to a position in the same index in the
 ucMACBuffers array.  If the index contains a 1 then the buffer within
 ucMACBuffers is in use, if it contains a 0 then the buffer is free. */
static unsigned char rxBufferInUse[NUM_RX_BUFFERS] = { 0 };
static unsigned char txBufferInUse[NUM_TX_BUFFERS] = { 0 };

/* If no buffers are available, then wait this long before looking again.... */
#define netifBUFFER_WAIT_DELAY					( 100 / portTICK_RATE_MS )

/* The semaphore used by the ISR to wake the lwIP task. */
xSemaphoreHandle ETH_RX_Sem = NULL;

/* Flag to indicate transmit machine is ready to fire off an outgoing packet */
static char s_tx_ready;

/* The lwip_buffer for Rx packet is not a fixed array, but instead gets pointed to the buffers
 allocated within this file. */
unsigned char * s_lwip_in_buf;

/* The lwip_buffer for Tx packet is not a fixed array, but instead gets pointed to the buffers
 allocated within this file. */
unsigned char * s_lwip_out_buf;

/* Index to the Rx descriptor to inspect next when looking for a received
 packet. */
static unsigned long s_ulNextDescriptor;


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
	while (RCC_GetFlagStatus(RCC_FLAG_PLL3RDY) == RESET);
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
		ETH_DMATxDescChainInit(xTxDescriptors, (void *) txMACBuffers, NUM_TX_DESCRIPTORS);
		ETH_DMARxDescChainInit(xRxDescriptors, (void *) rxMACBuffers, NUM_RX_DESCRIPTORS);
		for (ul = 0; ul < NUM_RX_DESCRIPTORS; ul++) {
			/* Ensure received data generates an interrupt. */
			ETH_DMARxDescReceiveITConfig(&(xRxDescriptors[ul]), ENABLE);
			/* Fix up the addresses used by the descriptors.
			 The way ETH_DMARxDescChainInit() is not compatible with the buffer
			 declarations in this file. */
			xRxDescriptors[ul].Buffer1Addr = (unsigned long) &(rxMACBuffers[ul][0]);
			/* Mark the buffer used by this descriptor as in use. */
			//rxBufferInUse[ul] = pdTRUE;
		}
		for (ul = 0; ul < NUM_TX_DESCRIPTORS; ul++) {
			/* Ensure received data generates an interrupt. */
			//ETH_DMATxDescTransmitITConfig(&(xTxDescriptors[ul]), ENABLE);
			/* Fix up the addresses used by the descriptors.
			 The way ETH_DMARxDescChainInit() is not compatible with the buffer
			 declarations in this file. */
			xTxDescriptors[ul].Buffer1Addr = (unsigned long) &(txMACBuffers[ul][0]);
			/* Mark the buffer used by this descriptor as in use. */
			//txBufferInUse[ul] = pdTRUE;
		}

		/* When receiving data, start at the first descriptor. */
		s_ulNextDescriptor = 0;
		/* Initialize s_lwip_buf to ensure it points somewhere valid. */
		s_lwip_in_buf = 0;
		s_lwip_out_buf = prvGetNextTXBuffer();
		/* Mark the tx machine as ready */
		s_tx_ready = 1;
		/* SendCount must be initialized to 2 to ensure the Tx descriptor looks
		 as if its available (as if it has already been sent twice. */
		//xTxDescriptor.SendCount = 2;
		// TODO: Enable hardware checksums
		/* Switch on the interrupts in the NVIC. */
		xNVICInit.NVIC_IRQChannel = ETH_IRQn;
		xNVICInit.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
		xNVICInit.NVIC_IRQChannelSubPriority = 0;
		xNVICInit.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&xNVICInit);

		/* Let the DMA know there are Rx descriptors available. */
		ETH->DMARPDR = 0;
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


unsigned char *prvGetNextRXBuffer(void) {
	portBASE_TYPE x;
	unsigned char *ucReturn = NULL;
	while (ucReturn == NULL) {
		/* Look through the buffers to find one that is not in use by
		 anything else. */
		vTaskSuspendAll();
		for (x = 0; x < NUM_RX_BUFFERS; x++) {
			if (rxBufferInUse[x] == pdFALSE) {
				rxBufferInUse[x] = pdTRUE;
				ucReturn = &(rxMACBuffers[x][0]);
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

unsigned char *prvGetNextTXBuffer(void) {
	portBASE_TYPE x;
	unsigned char *ucReturn = NULL;

	while (ucReturn == NULL) {
		/* Look through the buffers to find one that is not in use by
		 anything else. */
		vTaskSuspendAll();
		for (x = 0; x < NUM_TX_BUFFERS; x++) {
			if (txBufferInUse[x] == pdFALSE) {
				txBufferInUse[x] = pdTRUE;
				ucReturn = &(txMACBuffers[x][0]);
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
		xSemaphoreGiveFromISR( ETH_RX_Sem, &xHigherPriorityTaskWoken );
	}
	if (ulStatus & ETH_DMA_IT_T) {
		/* Data was transmitted, ready to transmit again */
		s_tx_ready = 1;
		/* The Tx buffer is no longer required. */
		vReturnTxBuffer((unsigned char *) xTxDescriptors[0].Buffer1Addr);
	}

	/* If xSemaphoreGiveFromISR() unblocked a task, and the unblocked task has
	 a higher priority than the currently executing task, then
	 xHigherPriorityTaskWoken will have been set to pdTRUE and this ISR should
	 return directly to the higher priority unblocked task. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void vReturnTxBuffer(unsigned char *pucBuffer) {
	unsigned long ul;

	/* Mark a buffer as free for use. */
	for (ul = 0; ul < NUM_TX_BUFFERS; ul++) {
		if (txMACBuffers[ul] == pucBuffer) {
			txBufferInUse[ul] = pdFALSE;
			break;
		}
	}
}

void vReturnRxBuffer(unsigned char *pucBuffer) {
	unsigned long ul;

	/* Mark a buffer as free for use. */
	for (ul = 0; ul < NUM_RX_BUFFERS; ul++) {
		if (rxMACBuffers[ul] == pucBuffer) {
			rxBufferInUse[ul] = pdFALSE;
			break;
		}
	}
}

void ETH_IRQHandler()
{
	vMAC_ISR();
}

void DMA2_Channel5_IRQHandler()
{
	vMAC_ISR();
}

unsigned char vSendMACData(unsigned short usDataLen) {
	unsigned long ulAttempts = 0UL;
	unsigned char res = 1;

	/* Check to see if the Tx descriptor is free. */
	while (!s_tx_ready || (xTxDescriptors[0].Status & ETH_DMATxDesc_OWN) == ETH_DMATxDesc_OWN) {
		/* Wait for the Tx descriptor to become available. */
		vTaskDelay(netifBUFFER_WAIT_DELAY);

		ulAttempts++;
		if (ulAttempts > netifBUFFER_WAIT_ATTEMPTS) {
			/* Something has gone wrong as the Tx descriptor is still in use.
			 Clear it down manually, the data it was sending will probably be
			 lost. */
			xTxDescriptors[0].Status &= ~ETH_DMATxDesc_OWN;
			vReturnTxBuffer((unsigned char *) xTxDescriptors[0].Buffer1Addr);
			res = 0;
			break;
		}
	}

	/* Setup the Tx descriptor for transmission. */
	s_tx_ready = 0;
	xTxDescriptors[0].Buffer1Addr = (unsigned long) s_lwip_out_buf;
	xTxDescriptors[0].ControlBufferSize = (unsigned long) usDataLen;
	xTxDescriptors[0].Status = ETH_DMATxDesc_OWN | ETH_DMATxDesc_LS
			| ETH_DMATxDesc_FS | ETH_DMATxDesc_TER | ETH_DMATxDesc_TCH
			| ETH_DMATxDesc_IC;
	ETH->DMASR = ETH_DMASR_TBUS;
	ETH->DMATPDR = 0;
	/* s_lwip_out_buf is being sent by the Tx descriptor.  Allocate a new buffer. */
	//TODO: It causes problems under load. It  seems it is not thread safe.
	s_lwip_out_buf = prvGetNextTXBuffer();
	vTaskDelay(netifBUFFER_WAIT_DELAY);
	return res;
}

unsigned short usGetMACRxData(void) {
	unsigned short usReturn;
	if ((xRxDescriptors[s_ulNextDescriptor].Status & ETH_DMARxDesc_ES) != 0) {
		/* Error in Rx.  Discard the frame and give it back to the DMA. */
		xRxDescriptors[s_ulNextDescriptor].Status = ETH_DMARxDesc_OWN;
		ETH->DMARPDR = 0;

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
			vReturnRxBuffer(s_lwip_in_buf);

		/* Get the received data length	from the top 2 bytes of the Status
		 word and the data itself. */
		usReturn = (unsigned short) ((xRxDescriptors[s_ulNextDescriptor].Status & ETH_DMARxDesc_FL) >> 16UL);
		s_lwip_in_buf = (unsigned char *) (xRxDescriptors[s_ulNextDescriptor].Buffer1Addr);

		/* Allocate a new buffer to the descriptor. */
		xRxDescriptors[s_ulNextDescriptor].Buffer1Addr = (unsigned long) prvGetNextRXBuffer();

		/* Give the descriptor back to the DMA. */
		xRxDescriptors[s_ulNextDescriptor].Status = ETH_DMARxDesc_OWN;
		ETH->DMARPDR = 0;

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
