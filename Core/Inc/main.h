/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void UsbDataReceived(uint8_t* pBuffer, uint16_t size);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/*
 * SPI1 interrupt callback (used for eeprom)
 */
void SpiInterruptCallback(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIPSW_1_Pin GPIO_PIN_14
#define DIPSW_1_GPIO_Port GPIOC
#define DIPSW_2_Pin GPIO_PIN_15
#define DIPSW_2_GPIO_Port GPIOC
#define LED_USER_R_Pin GPIO_PIN_0
#define LED_USER_R_GPIO_Port GPIOC
#define LED_USER_G_Pin GPIO_PIN_1
#define LED_USER_G_GPIO_Port GPIOC
#define LED_CAN1_R_Pin GPIO_PIN_2
#define LED_CAN1_R_GPIO_Port GPIOC
#define LED_CAN1_Y_Pin GPIO_PIN_3
#define LED_CAN1_Y_GPIO_Port GPIOC
#define LED_CAN2_R_Pin GPIO_PIN_0
#define LED_CAN2_R_GPIO_Port GPIOA
#define LED_CAN2_Y_Pin GPIO_PIN_1
#define LED_CAN2_Y_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_2
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_3
#define DEBUG_RX_GPIO_Port GPIOA
#define SPI_MEM_CS_Pin GPIO_PIN_4
#define SPI_MEM_CS_GPIO_Port GPIOA
#define SPI_MEM_SCK_Pin GPIO_PIN_5
#define SPI_MEM_SCK_GPIO_Port GPIOA
#define SPI_MEM_MISO_Pin GPIO_PIN_6
#define SPI_MEM_MISO_GPIO_Port GPIOA
#define SPI_MEM_MOSI_Pin GPIO_PIN_7
#define SPI_MEM_MOSI_GPIO_Port GPIOA
#define OUT_3_OD_Pin GPIO_PIN_4
#define OUT_3_OD_GPIO_Port GPIOC
#define OUT_2_PP_Pin GPIO_PIN_5
#define OUT_2_PP_GPIO_Port GPIOC
#define ADIN_1_Pin GPIO_PIN_0
#define ADIN_1_GPIO_Port GPIOB
#define ADIN_2_Pin GPIO_PIN_1
#define ADIN_2_GPIO_Port GPIOB
#define OUT_1_PP_Pin GPIO_PIN_2
#define OUT_1_PP_GPIO_Port GPIOB
#define SD_DETECT_Pin GPIO_PIN_10
#define SD_DETECT_GPIO_Port GPIOB
#define __5V_SENS_Pin GPIO_PIN_11
#define __5V_SENS_GPIO_Port GPIOB
#define SPI_SD_CS_Pin GPIO_PIN_12
#define SPI_SD_CS_GPIO_Port GPIOB
#define SPI_SD_SCK_Pin GPIO_PIN_13
#define SPI_SD_SCK_GPIO_Port GPIOB
#define SPI_SD_MISO_Pin GPIO_PIN_14
#define SPI_SD_MISO_GPIO_Port GPIOB
#define SPI_SD_MOSI_Pin GPIO_PIN_15
#define SPI_SD_MOSI_GPIO_Port GPIOB
#define _5V_OUT_EN_Pin GPIO_PIN_7
#define _5V_OUT_EN_GPIO_Port GPIOC
#define _5V_OUT_FAULT_Pin GPIO_PIN_8
#define _5V_OUT_FAULT_GPIO_Port GPIOC
#define CAN2_STB_Pin GPIO_PIN_9
#define CAN2_STB_GPIO_Port GPIOC
#define CAN2_RXD_Pin GPIO_PIN_8
#define CAN2_RXD_GPIO_Port GPIOA
#define UART_TXD_Pin GPIO_PIN_9
#define UART_TXD_GPIO_Port GPIOA
#define UART_RXD_Pin GPIO_PIN_10
#define UART_RXD_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define CAN2_TXD_Pin GPIO_PIN_15
#define CAN2_TXD_GPIO_Port GPIOA
#define LIN_TXD_Pin GPIO_PIN_10
#define LIN_TXD_GPIO_Port GPIOC
#define LIN_RXD_Pin GPIO_PIN_11
#define LIN_RXD_GPIO_Port GPIOC
#define LIN_CS_Pin GPIO_PIN_12
#define LIN_CS_GPIO_Port GPIOC
#define CAN1_STB_Pin GPIO_PIN_2
#define CAN1_STB_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define CAN1_RXD_Pin GPIO_PIN_5
#define CAN1_RXD_GPIO_Port GPIOB
#define CAN1_TXD_Pin GPIO_PIN_6
#define CAN1_TXD_GPIO_Port GPIOB
#define LIN_WAKE_Pin GPIO_PIN_7
#define LIN_WAKE_GPIO_Port GPIOB
#define RESET_USER_Pin GPIO_PIN_8
#define RESET_USER_GPIO_Port GPIOB
#define RESET_USER_EXTI_IRQn EXTI9_5_IRQn
#define LIN_MASTER_Pin GPIO_PIN_9
#define LIN_MASTER_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/* User LED controls */
#define LED_G_TOGGLE()      HAL_GPIO_TogglePin(LED_USER_G_GPIO_Port, LED_USER_G_Pin)
#define LED_G_ON()          HAL_GPIO_WritePin(LED_USER_G_GPIO_Port, LED_USER_G_Pin, 1)
#define LED_G_OFF()         HAL_GPIO_WritePin(LED_USER_G_GPIO_Port, LED_USER_G_Pin, 0)

#define LED_R_TOGGLE()      HAL_GPIO_TogglePin(LED_USER_R_GPIO_Port, LED_USER_R_Pin)
#define LED_R_ON()          HAL_GPIO_WritePin(LED_USER_R_GPIO_Port, LED_USER_R_Pin, 1)
#define LED_R_OFF()         HAL_GPIO_WritePin(LED_USER_R_GPIO_Port, LED_USER_R_Pin, 0)

/* CAN LED control */
#define LED_CAN1_R_ON()     HAL_GPIO_WritePin(LED_CAN1_R_GPIO_Port, LED_CAN1_R_Pin, 1)
#define LED_CAN1_R_OFF()    HAL_GPIO_WritePin(LED_CAN1_R_GPIO_Port, LED_CAN1_R_Pin, 0)
#define LED_CAN1_Y_ON()     HAL_GPIO_WritePin(LED_CAN1_Y_GPIO_Port, LED_CAN1_Y_Pin, 1)
#define LED_CAN1_Y_OFF()    HAL_GPIO_WritePin(LED_CAN1_Y_GPIO_Port, LED_CAN1_Y_Pin, 0)

#define LED_CAN2_R_ON()     HAL_GPIO_WritePin(LED_CAN2_R_GPIO_Port, LED_CAN2_R_Pin, 1)
#define LED_CAN2_R_OFF()    HAL_GPIO_WritePin(LED_CAN2_R_GPIO_Port, LED_CAN2_R_Pin, 0)
#define LED_CAN2_Y_ON()     HAL_GPIO_WritePin(LED_CAN2_Y_GPIO_Port, LED_CAN2_Y_Pin, 1)
#define LED_CAN2_Y_OFF()    HAL_GPIO_WritePin(LED_CAN2_Y_GPIO_Port, LED_CAN2_Y_Pin, 0)

/* CAN transceiver control */
#define PIN_CAN1_STDBY_ON() HAL_GPIO_WritePin(CAN1_STB_GPIO_Port, CAN1_STB_Pin, 0)
#define PIN_CAN2_STDBY_ON() HAL_GPIO_WritePin(CAN2_STB_GPIO_Port, CAN2_STB_Pin, 0)

/* Digital inputs */
#define RESET_USER_READ()   HAL_GPIO_ReadPin(RESET_USER_GPIO_Port, RESET_USER_Pin)
#define DIPSW1_READ()       HAL_GPIO_ReadPin(DIPSW_1_GPIO_Port, DIPSW_1_Pin)
#define DIPSW2_READ()       HAL_GPIO_ReadPin(DIPSW_2_GPIO_Port, DIPSW_2_Pin)
#define _5V_SENSE_READ()    HAL_GPIO_ReadPin(__5V_SENS_GPIO_Port, __5V_SENS_Pin)

/* Digital outputs */
#define OUT1_ON()           HAL_GPIO_WritePin(OUT_1_PP_GPIO_Port, OUT_1_PP_Pin, 1)      /* DO2 */
#define OUT1_OFF()          HAL_GPIO_WritePin(OUT_1_PP_GPIO_Port, OUT_1_PP_Pin, 0)
#define OUT2_ON()           HAL_GPIO_WritePin(OUT_2_PP_GPIO_Port, OUT_2_PP_Pin, 1)      /* DO3 */
#define OUT2_OFF()          HAL_GPIO_WritePin(OUT_2_PP_GPIO_Port, OUT_2_PP_Pin, 0)
#define OUT3_ON()           HAL_GPIO_WritePin(OUT_3_OD_GPIO_Port, OUT_3_OD_Pin, 1)      /* DO4 */
#define OUT3_OFF()          HAL_GPIO_WritePin(OUT_3_OD_GPIO_Port, OUT_3_OD_Pin, 0)
#define _5V_ON()            HAL_GPIO_WritePin(_5V_OUT_EN_GPIO_Port, _5V_OUT_EN_Pin, 1)  /* DO1 */
#define _5V_OFF()           HAL_GPIO_WritePin(_5V_OUT_EN_GPIO_Port, _5V_OUT_EN_Pin, 0)

/* Eeprom chip select outputs */
#define RESET_EE_CS_PIN()   HAL_GPIO_WritePin(SPI_MEM_CS_GPIO_Port, SPI_MEM_CS_Pin, 0)
#define SET_EE_CS_PIN()     HAL_GPIO_WritePin(SPI_MEM_CS_GPIO_Port, SPI_MEM_CS_Pin, 1)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
