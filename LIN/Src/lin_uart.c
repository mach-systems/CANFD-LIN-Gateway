/*
 * lin_uart.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Karel Hevessy
 */

#include <stm32g4xx_ll_usart.h>
#include <stm32g4xx_ll_rcc.h>     /* LL_RCC_GetUSARTClockFreq() */
#include "lin_uart.h"
#include "lin.h"

void InitLinUartBaudrate(void)
{
  /* Only thing needed is changing of baudrate, the rest is managed by STM32 Device Configuration Tool */
  LL_USART_Disable(LIN_UART);
  uint32_t baudrate;

  if(_LIN_SPEED_BPS_ == 9600)
    baudrate = 9600;
  else if(_LIN_SPEED_BPS_ == 19200)
    baudrate = 19200;
  else /* Default value */
    baudrate = 0;

  uint32_t periphclk = LL_RCC_PERIPH_FREQUENCY_NO;
  if (LIN_UART == USART3)
    periphclk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART3_CLKSOURCE);

  if (periphclk != LL_RCC_PERIPH_FREQUENCY_NO && baudrate != 0)
    LL_USART_SetBaudRate(LIN_UART, periphclk, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, baudrate);
  LL_USART_Enable(LIN_UART);
}

void LinUartIrqHandler(void)
{
  if (LL_USART_IsActiveFlag_LBD(LIN_UART) && LL_USART_IsEnabledIT_LBD(LIN_UART))
  {
    LL_USART_ClearFlag_LBD(LIN_UART);
    LinLineBreakCallback();
  }

  if (LL_USART_IsActiveFlag_FE(LIN_UART) && LL_USART_IsEnabledIT_ERROR(LIN_UART))
  {
    LL_USART_ClearFlag_FE(LIN_UART);
    LinFramingErrorCallback();
  }

  if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART3) && LL_USART_IsEnabledIT_RXNE_RXFNE(LIN_UART))
  {
    /* Rxne interrupt flag cleared by reading from the buffer */
    uint8_t rxByte = LL_USART_ReceiveData8(LIN_UART);
    LinRxCallback(rxByte);
  }

  if (LL_USART_IsActiveFlag_TXE_TXFNF(LIN_UART) && LL_USART_IsEnabledIT_TXE_TXFNF(LIN_UART))
  {
    /* Txe interrupt flag cleared by writing to the buffer */
    if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART3))
      while (1);

    LinTxCallback();
  }

}

uint8_t LinUartGetChar(void)
{
  return LL_USART_ReceiveData8(LIN_UART);
}

void LinUartPutChar(uint8_t charToSend)
{
  LL_USART_TransmitData8(LIN_UART, charToSend);
}

void LinUartReset(void)
{
  LL_USART_Disable(LIN_UART);
  LL_USART_Enable(LIN_UART);
}

void LinUartEnableTxInterrupt(void)
{
  LL_USART_EnableIT_TXE(LIN_UART);
}

void LinUartDisableTxInterrupt(void)
{
  LL_USART_DisableIT_TXE(LIN_UART);
  // todo
  LL_USART_RequestRxDataFlush(LIN_UART);
}

void LinUartEnableRxInterrupt(void)
{
  LL_USART_EnableIT_RXNE_RXFNE(LIN_UART);
}

void LinUartDisableRxInterrupt(void)
{
  LL_USART_DisableIT_RXNE_RXFNE(LIN_UART);
}

void LinUartEnableBreakInterrupt(void)
{
  LL_USART_EnableIT_LBD(LIN_UART);
}

void LinUartDisableBreakInterrupt(void)
{
  LL_USART_DisableIT_LBD(LIN_UART);
}

void LinUartSendBreak(void)
{
  LL_USART_RequestBreakSending(LIN_UART);
}
