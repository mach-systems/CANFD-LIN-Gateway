/*
 * bsp_io.h
 *
 *  Created on: Sep 2, 2020
 *      Author: Karel Hevessy
 */

#ifndef BSP_IO_H_
#define BSP_IO_H_

#include <stdint.h>

/* bus.h: */
#ifndef BUS_SPI2_POLL_TIMEOUT
  #define BUS_SPI2_POLL_TIMEOUT            0x1000U
#endif

/* from stm32g474e_eval_bus.h */
/**
  * @brief  Write Data through SPI BUS.
  * @param  pData  Pointer to data buffer to send
  * @param  Length Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI2_Send(uint8_t *pData, uint16_t Length);

/**
  * @brief  Receive Data from SPI BUS
  * @param  pData  Pointer to data buffer to receive
  * @param  Length Length of data in byte
  * @retval BSP status
  */
int32_t  BSP_SPI2_Recv(uint8_t *pData, uint16_t Length);

/**
  * @brief  Send and Receive data to/from SPI BUS (Full duplex)
  * @param  pTxData  Pointer to data buffer to send
  * @param  pRxData  Pointer to data buffer to receive
  * @param  Length   Length of data in byte
  * @retval BSP status
  */
int32_t BSP_SPI2_SendRecv(uint8_t *pTxData, uint8_t *pRxData, uint16_t Length);



#endif /* BSP_IO_H_ */
