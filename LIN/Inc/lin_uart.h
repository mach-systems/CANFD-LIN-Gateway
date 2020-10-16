/*
 * lin_uart.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Karel Hevessy
 */

#ifndef INC_LIN_UART_H_
#define INC_LIN_UART_H_


/*
 * Change Uart baudrate according to define in lin.h
 */
void InitLinUartBaudrate(void);


/*
 * Uart interrupt handler (LL driver has one callback for all UART interrupts).
 */
void LinUartIrqHandler(void);

/*
 * Read the received character from UART.
 */
uint8_t LinUartGetChar(void);

/*
 * Write character to UART buffer (and init the transmission by this).
 */
void LinUartPutChar(uint8_t charToSend);

/*
 * Reset the UART peripheral.
 */
void LinUartReset(void);

/*
 * Enable the UART tx empty interrupt.
 */
void LinUartEnableTxInterrupt(void);

/*
 * Disable the UART tx empty interrupt.
 */
void LinUartDisableTxInterrupt(void);

/*
 * Enable the UART rx not empty interrupt.
 */
void LinUartEnableRxInterrupt(void);

/*
 * Disable the UART rx not empty interrupt.
 */
void LinUartDisableRxInterrupt(void);

/*
 * Enable the break character detection interrupt.
 */
void LinUartEnableBreakInterrupt(void);

/*
 * Disable break character detection interrupt.
 */
void LinUartDisableBreakInterrupt(void);

/*
 * Request sending of break character.
 */
void LinUartSendBreak();



#endif /* INC_LIN_UART_H_ */
