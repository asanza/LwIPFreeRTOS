/*
 * beth.h
 *
 *  Created on: 26.08.2012
 *      Author: diego
 */

#ifndef BETH_H_
#define BETH_H_

#include <lwip/err.h>
#include <lwip/netif.h>
#include <stdint.h>
#include <stm32_eth.h>
/// Board physical addresse
#define PHY_ADDRESS  1

typedef struct {
	uint32_t length;
	uint32_t buffer;
	ETH_DMADESCTypeDef *descriptor;
}FrameTypeDef;


err_t beth_initialize(struct netif *netif);
err_t beth_wait_packet();
uint32_t ETH_GetCurrentTxBuffer(void);
err_t ETH_TxPkt_ChainMode(uint16_t FrameLength);
FrameTypeDef ETH_RxPkt_ChainMode(void);


#endif /* BETH_H_ */
