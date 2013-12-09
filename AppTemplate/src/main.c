/** @file main.c
 *  Main Entry Point.
 *  Managing Complexity Practices.
 *  Simple WebServer with lots of services...
 *
 *  (c) 2013 Diego F. Asanza.
 */

#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "hwsetup.h"

/** Setup the hardware*/
static void prvSetupHardware( void );
/*set task priorities. Cortex-M3 high priority is zero.*/
#define mainTaskPriority			( configMAX_PRIORITIES) /// it really the lowest priority
#define slaveTaskPriority			( configMAX_PRIORITIES) /// the same

/*generate two simple tasks*/
static void mainTask( void *pvparameters );
static void slaveTask(void *pvparameters);
/*entry point*/
int main( void )
{
	prvSetupHardware();
	SystemInit();
	init_usart();
	//tcpip_init( prvEthernetConfigureInterface, NULL );

	/* Retarget the C library printf function to the USARTx, can be USART1 or USART2
	depending on the EVAL board you are using ********************************/
	printf("Init Cuasi-complete\n\r");
	//printf("\n\rOLIMEX STM32P107 - ETHERNET DEMO\n\r");

	xTaskCreate(mainTask,( signed char * )"MAIN",
			configMINIMAL_STACK_SIZE * 10,
			NULL, mainTaskPriority, NULL );

	xTaskCreate(slaveTask,( signed char * )"SLAVE",
			configMINIMAL_STACK_SIZE * 10,
			NULL, mainTaskPriority, NULL );

	//sys_thread_new("ECHO",EchoServer,(void*)NULL, 1800, mainTaskPriority); /// low priority task

	/* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );
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
		vTaskDelay(30);
	}
}

//void ETH_WKUP_IRQHandler()
//{
//	while(1);
//}


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
