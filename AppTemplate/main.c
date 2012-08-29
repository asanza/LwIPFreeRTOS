/**/


/**/
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "hwsetup.h"

static void prvSetupHardware( void );
/// TODO: Dont forget, for cortex m3 0 is high priority, 255 low priority
//Tasks Priorities
#define mainTaskPriority			( configMAX_PRIORITIES) /// it really the lowest priority
#define slaveTaskPriority			( configMAX_PRIORITIES) /// the same


static void mainTask( void *pvparameters );
static void slaveTask(void *pvparameters);

#include <lwip/tcpip.h>
#include <lwip/ip_addr.h>

static void prvEthernetConfigureInterface(void * param);
static void EchoRequest( struct netconn *pxNetCon );
portTASK_FUNCTION( EchoServer, pvParameters );

int main( void )
{
	prvSetupHardware();
	SystemInit();
	init_usart();
	tcpip_init( prvEthernetConfigureInterface, NULL );

	/* Retarget the C library printf function to the USARTx, can be USART1 or USART2
	depending on the EVAL board you are using ********************************/
	//printf("\n\rOLIMEX STM32P107 - ETHERNET DEMO\n\r");

	xTaskCreate(mainTask,( signed char * )"MAIN",
			configMINIMAL_STACK_SIZE * 10,
			NULL, mainTaskPriority, NULL );

	xTaskCreate(slaveTask,( signed char * )"SLAVE",
			configMINIMAL_STACK_SIZE * 10,
			NULL, mainTaskPriority, NULL );

	sys_thread_new("ECHO",EchoServer,(void*)NULL, 1800, mainTaskPriority); /// low priority task

	/* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );
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
                        netconn_write( pxNetCon, pcRxString, (u16_t) usLength, NETCONN_COPY );
                        netconn_write( pxNetCon, "Hello Internet!\n", (u16_t) 16, NETCONN_COPY);
                        printf("REC: %s",pcRxString);
                }
                netbuf_delete( pxRxBuffer );
        }
}


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

static void mainTask( void *pvparameters )
{
	STM_EVAL_LEDInit(LED1);
	while(1)
	{
		STM_EVAL_LEDToggle(LED1);
		vTaskDelay(10);
	}
}


static void slaveTask( void *pvparameters )
{
	STM_EVAL_LEDInit(LED2);
	STM_EVAL_LEDInit(LED4);
	while(1)
	{
		STM_EVAL_LEDToggle(LED2);
		//STM_EVAL_LEDToggle(LED4);
		vTaskDelay(10);
	}
}

//void ETH_WKUP_IRQHandler()
//{
//	while(1);
//}

void HardFault_Handler( void )
{
__ASM("TST LR, #4");
__ASM("ITE EQ");
__ASM("MRSEQ R0, MSP");
__ASM("MRSNE R0, PSP");
__ASM("B hard_fault_handler_c");
}

void hard_fault_handler_c (unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  printf ("\n\n[Hard fault handler - all numbers in hex]\n");
  printf ("R0 = %x\n", stacked_r0);
  printf ("R1 = %x\n", stacked_r1);
  printf ("R2 = %x\n", stacked_r2);
  printf ("R3 = %x\n", stacked_r3);
  printf ("R12 = %x\n", stacked_r12);
  printf ("LR [R14] = %x  subroutine call return address\n", stacked_lr);
  printf ("PC [R15] = %x  program counter\n", stacked_pc);
  printf ("PSR = %x\n", stacked_psr);
  printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
  printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
  printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
  printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
  printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
  printf ("SCB_SHCSR = %x\n", SCB->SHCSR);

  while (1);
}

/*-----------------------------------------------------------*/


static void prvSetupHardware( void )
{
	/* 2 wait states required on the flash. */
	*( ( unsigned long * ) 0x40022000 ) = 0x02;

}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
unsigned ntask;
unsigned oldntask = 0;

void vApplicationTickHook( void )
{
	ntask = uxTaskGetNumberOfTasks();
	if(oldntask!=ntask)
	{
		printf("Running %d tasks\n",ntask);
		oldntask = ntask;
	}
}


/*-----------------------------------------------------------*/



void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	( void ) pxTask;
	( void ) pcTaskName;
	usart_putstr("Overflow\n");
	for( ;; )
	{
		portNOP();
	};
}
