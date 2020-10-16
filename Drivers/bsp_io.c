/*
 * bsp_io.c
 *
 *  Created on: Sep 2, 2020
 *      Author: Karel Hevessy
 */


#include "bsp_io.h"
#include "main.h"
#include "stm32g474re_errno.h"

/* stm32g474e_eval_bus.h: */
#ifndef BUS_SPI2_POLL_TIMEOUT
  #define BUS_SPI2_POLL_TIMEOUT            0x1000U
#endif

/**
  * @brief  Write Data through SPI BUS.
  * @param  pData  Pointer to data buffer to send
  * @param  Length Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI2_Send(uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(HAL_SPI_Transmit(&hspi2, pData, Length, BUS_SPI2_POLL_TIMEOUT) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }
  return ret;
}

/**
  * @brief  Receive Data from SPI BUS
  * @param  pData  Pointer to data buffer to receive
  * @param  Length Length of data in byte
  * @retval BSP status
  */
int32_t  BSP_SPI2_Recv(uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;
  uint32_t tx_data = 0xFFFFFFFFU;

  if(HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&tx_data, pData, Length, BUS_SPI2_POLL_TIMEOUT) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }
  return ret;
}

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pTxData  Pointer to data buffer to send
  * @param  pRxData  Pointer to data buffer to receive
  * @param  Length   Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI2_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_UNKNOWN_FAILURE;

  if(HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, Length, BUS_SPI2_POLL_TIMEOUT) == HAL_OK)
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}
