/**
  ******************************************************************************
  * @file    stm3210c_eval.h
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   This file contains definitions for STM3210C_EVAL's Leds, push-buttons
  *          and COM ports hardware resources.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210C_EVAL_H
#define __STM3210C_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup STM3210C_EVAL
  * @{
  */ 


/** @defgroup STM3210C_EVAL_Exported_Types
  * @{
  */
/**
  * @}
  */ 

/** @defgroup STM3210C_EVAL_Exported_Constants
  * @{
  */ 

/** @addtogroup STM3210C_EVAL_LED
  * @{
  */
#define LEDn                        4
#define LED1_GPIO_PORT              GPIOC
#define LED1_GPIO_CLK               RCC_APB2Periph_GPIOC  
#define LED1_GPIO_PIN               GPIO_Pin_6
  
#define LED2_GPIO_PORT              GPIOC
#define LED2_GPIO_CLK               RCC_APB2Periph_GPIOC  
#define LED2_GPIO_PIN               GPIO_Pin_7

#define LED3_GPIO_PORT              GPIOC
#define LED3_GPIO_CLK               RCC_APB2Periph_GPIOC
#define LED3_GPIO_PIN               GPIO_Pin_10

#define LED4_GPIO_PORT              GPIOE
#define LED4_GPIO_CLK               RCC_APB2Periph_GPIOE
#define LED4_GPIO_PIN               GPIO_Pin_7

  
/**
  * @}
  */ 
  
/** @addtogroup STM3210C_EVAL_BUTTON
  * @{
  */  
#define BUTTONn                     2 /**/

/**
 * @brief Wakeup push-button
 */
#define WAKEUP_BUTTON_PORT          GPIOA
#define WAKEUP_BUTTON_CLK           RCC_APB2Periph_GPIOA
#define WAKEUP_BUTTON_PIN           GPIO_Pin_0
#define WAKEUP_BUTTON_EXTI_LINE     EXTI_Line0
#define WAKEUP_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOA
#define WAKEUP_BUTTON_PIN_SOURCE    GPIO_PinSource0
#define WAKEUP_BUTTON_IRQn          EXTI0_IRQn 

/**
 * @brief Tamper push-button
 */
#define TAMPER_BUTTON_PORT          GPIOC
#define TAMPER_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define TAMPER_BUTTON_PIN           GPIO_Pin_13
#define TAMPER_BUTTON_EXTI_LINE     EXTI_Line13
#define TAMPER_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOC
#define TAMPER_BUTTON_PIN_SOURCE    GPIO_PinSource13
#define TAMPER_BUTTON_IRQn          EXTI15_10_IRQn 

/**
  * @}
  */ 

/** @addtogroup STM3210C_EVAL_COM
  * @{
  */
#define COMn                        1

/**
 * @brief Definition for COM port1, connected to USART3 
 */ 
#define EVAL_COM1                   USART2
#define EVAL_COM1_GPIO              GPIOD
#define EVAL_COM1_CLK               RCC_APB1Periph_USART2
#define EVAL_COM1_GPIO_CLK          RCC_APB2Periph_GPIOD
#define EVAL_COM1_RxPin             GPIO_Pin_6
#define EVAL_COM1_TxPin             GPIO_Pin_5
#define EVAL_COM1_CTSPin            GPIO_Pin_11
#define EVAL_COM1_RTSPin            GPIO_Pin_12

//#define 	COMn   1
//#define 	EVAL_COM1   USART2
// 	Definition for COM port1, connected to USART2 (USART2 pins remapped on GPIOD)
//#define 	EVAL_COM1_CLK   RCC_APB1Periph_USART2
//#define 	EVAL_COM1_IRQn   USART2_IRQn
//#define 	EVAL_COM1_RX_GPIO_CLK   RCC_APB2Periph_GPIOD
//#define 	EVAL_COM1_RX_GPIO_PORT   GPIOD
//#define 	EVAL_COM1_RX_PIN   GPIO_Pin_6
//#define 	EVAL_COM1_TX_GPIO_CLK   RCC_APB2Periph_GPIOD
//#define 	EVAL_COM1_TX_GPIO_PORT   GPIOD
//#define 	EVAL_COM1_TX_PIN   GPIO_Pin_5

/**
 * @brief MMC/SD Card Card chip select
 */
#define MMC_CS_PORT                 GPIOA
#define MMC_CS_CLK                  RCC_APB2Periph_GPIOA
#define MMC_CS_PIN                  GPIO_Pin_4
#define MMC_CS_EXTI_LINE            EXTI_Line4
#define MMC_CS_PORT_SOURCE          GPIO_PortSourceGPIOA
#define MMC_CS_PIN_SOURCE           GPIO_PinSource4
#define MMC_CS_IRQn                 EXTI4_IRQn

/**
 * @brief MMC/SD Card SPI
 */
#define MMC_SPI                SPI3   /* SPI pins are remapped by software */
#define MMC_SPI_CLK            RCC_APB1Periph_SPI3
#define MMC_SPI_GPIO           GPIOC
#define MMC_SPI_GPIO_CLK       RCC_APB2Periph_GPIOC  
#define MMC_PIN_SCK            GPIO_Pin_10
#define MMC_PIN_MISO           GPIO_Pin_11
#define MMC_PIN_MOSI           GPIO_Pin_12

/**
 * @brief USB_VBUSON
 */
#define USB_VBUSON_PORT                 GPIOD
#define USB_VBUSON_CLK                  RCC_APB2Periph_GPIOD
#define USB_VBUSON_PIN                  GPIO_Pin_7

/**
 * @brief USB_FAULT 
 */
#define USB_FAULT_PORT                 GPIOE
#define USB_FAULT_CLK                  RCC_APB2Periph_GPIOE
#define USB_FAULT_PIN                  GPIO_Pin_15
#define USB_FAULT_EXTI_LINE            EXTI_Line15
#define USB_FAULT_PORT_SOURCE          GPIO_PortSourceGPIOE
#define USB_FAULT_PIN_SOURCE           GPIO_PinSource15
#define USB_FAULT_IRQn                 EXTI15_10_IRQn

/**
 * @brief USB_FAULT 
 */
#define USB_OTGID_PORT                 GPIOA
#define USB_OTGID_CLK                  RCC_APB2Periph_GPIOA
#define USB_OTGID_PIN                  GPIO_Pin_10
#define USB_OTGID_EXTI_LINE            EXTI_Line10
#define USB_OTGID_PORT_SOURCE          GPIO_PortSourceGPIOA
#define USB_OTGID_PIN_SOURCE           GPIO_PinSource10
#define USB_OTGID_IRQn                 EXTI15_10_IRQn

 /*
#define EVAL_COM1                   USART3
#define EVAL_COM1_GPIO              GPIOD
#define EVAL_COM1_CLK               RCC_APB1Periph_USART3
#define EVAL_COM1_GPIO_CLK          RCC_APB2Periph_GPIOD
#define EVAL_COM1_RxPin             GPIO_Pin_9
#define EVAL_COM1_TxPin             GPIO_Pin_8
#define EVAL_COM1_CTSPin            GPIO_Pin_11
#define EVAL_COM1_RTSPin            GPIO_Pin_12
*/
   /**
  * @}
  */ 

/**
  * @}
  */ 
  
/** @defgroup STM3210C_EVAL_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210C_EVAL_Exported_Functions
  * @{
  */ 
/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __STM3210C_EVAL_H */
/**
  * @}
  */ 


/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
