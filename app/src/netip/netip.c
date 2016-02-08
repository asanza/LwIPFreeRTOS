/*
 * netip.c
 *
 *  Created on: Dec 8, 2013
 *      Author: diego
 */

#include <stdint.h>
#include <stdio.h>
#include <lwip/tcpip.h>
#include <lwip/ip_addr.h>

static struct netif s_EMAC_if;
static void prvEthernetConfigureInterface(void * param)
{
	struct ip_addr xIpAddr, xNetMast, xGateway;
	extern err_t ethernetif_init( struct netif *netif );
	/* Parameters are not used - suppress compiler error. */
	( void ) param;
	/* Create and configure the EMAC interface. */
	IP4_ADDR( &xIpAddr, 192, 168, 1, 1 );
	IP4_ADDR( &xNetMast, 255, 255, 255, 0 );
	IP4_ADDR( &xGateway, 192, 168, 1, 71 );
	netif_add( &s_EMAC_if, &xIpAddr, &xNetMast, &xGateway, NULL, ethernetif_init, tcpip_input );
	// comment this line to use DHCP
	netif_set_up(&s_EMAC_if);
	/* make it the default interface */
	netif_set_default(&s_EMAC_if);
}

portTASK_FUNCTION( EchoServer, pvParameters ){
        struct netconn *pxTCPListener, *pxNewConnection;
        pxTCPListener = netconn_new( NETCONN_TCP );
        netconn_bind(pxTCPListener, NULL, 23 );
        netconn_listen(pxTCPListener );
        for( ;; ){
                err_t error = netconn_accept(pxTCPListener,&pxNewConnection);
                if(error == ERR_OK){
//                	while(1)
                        EchoRequest(pxNewConnection);
                }
                netconn_close(pxNewConnection);
                netconn_delete(pxNewConnection);
        }
}

void EchoRequest( struct netconn *pxNetCon ) {
        struct netbuf *pxRxBuffer;
        portCHAR *pcRxString;
        unsigned portSHORT usLength;
        netconn_recv( pxNetCon, &pxRxBuffer);
        if ( pxRxBuffer != NULL ){
                netbuf_data( pxRxBuffer, ( void * ) &pcRxString, &usLength );
                if (  pcRxString != NULL){
                        netconn_write( pxNetCon, pcRxString, (uint16_t) usLength, NETCONN_COPY );
                        netconn_write( pxNetCon, "Hello Internet!\n", (uint16_t) 16, NETCONN_COPY);
                        printf("REC: %s",pcRxString);
                }
                netbuf_delete( pxRxBuffer );
        }
}

