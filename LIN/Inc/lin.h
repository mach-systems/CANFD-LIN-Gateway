/*
 * lin.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Karel Hevessy
 */

#ifndef INC_LIN_H_
#define INC_LIN_H_

#include <stdint.h>

#define LIN_MASTER_MODE
/* Slave mode not implemented */

/*
 * Hardware defines
 */
#define _LIN_SPEED_BPS_       19200
#define LIN_UART              USART3

/*
 * LIN frame defines
 */
#define LIN_DATA_LENGTH       8

#define CLASSICAL_CHECKSUM    0
#define ENHANCED_CHECKSUM     1

#define MASTER_REQUEST        1
#define MASTER_RESPONSE       2

#define MAX_SCHEDULER_LENGTH  10
#define RX_BUFFER_LENGTH      32

/* Timeout of receiving the break character */
#define BREAK_TIMEOUT_MAX     1000

/*
 * LIN frame structure
 */
typedef struct
{
    uint8_t ID;
    uint8_t Data[LIN_DATA_LENGTH];
    uint8_t Length;
    uint8_t Checksum;
    uint8_t ChecksumType;
    uint8_t Type;
    uint8_t ResponseLength;
} LIN_Frame;

/*
 * LIN scheduler row
 */
typedef struct
{
  LIN_Frame Frame;
  uint16_t Delay;
} SchedulingTableRow;

/*
 * LIN scheduler structure
 */
typedef struct
{
  SchedulingTableRow Row[MAX_SCHEDULER_LENGTH];
  uint8_t NumberOfFrames;
} LinScheduler;

/*
 * LIN transmission states
 */
typedef enum
{
  STATE_SYNC_BREAK,
  STATE_SYNC_FIELD,
  STATE_ID,
  STATE_DATA,
  STATE_CHECKSUM,
  STATE_END
} LinState;

/*
 * Should be called with period of 1 ms.
 */
void SchedulerTimerCallback(void);

/*
 * Start the LIN scheduler.
 */
void LinSchedulerStart(void);

/*
 * Stop the LIN scheduler.
 */
void LinSchedulerStop(void);

/*
 * Initialize the LIN hardware and internal variables.
 */
void InitLin(void);

/*
 * Initialize the (hard coded) scheduler.
 */
void initLinScheduler(void);

/*
 * Begin transmission of the frame.
 */
uint8_t LinTransmitFrameAsync(LIN_Frame* pFrame);

/*
 * Timeout timer period elapsed.
 */
void LinTimeoutTimerCallback(void);

/*
 * Callback for UART rx interrupt.
 */
void LinRxCallback(uint8_t byteReceived);

/*
 * Callback for UART tx interrupt.
 */
void LinTxCallback(void);

/*
 * Callback for UART line break detection.
 */
void LinLineBreakCallback(void);

/*
 * Reset the LIN uart peripheral and all the state variables.
 */
void linResetAll(void);

/*
 * Reset all the state variables
 */
void resetStateMachine(void);

/*
 * Callback for UART framing error.
 */
void LinFramingErrorCallback(void);

/*
 * Calculate the LIN PID.
 */
uint8_t getParityIdentifier(uint8_t id);

/*
 * Calculate the checksum
 */
uint8_t calcChecksum(uint8_t address[], uint8_t size, uint8_t checksumType,
                     uint8_t linId);

/*
 * Calculate timeout for the slave response and return it.
 */
uint16_t linCalcResponseTimeout(uint8_t numberOfBytes);

/*
 * Update data of Master Response frame with ID. Return 1 if success, 0 if frame
 * could not be found.
 */
uint8_t UpdateTxFrameData(uint8_t ID, uint8_t* pData, uint8_t size);

/*
 * Try to find the Master Request frame with ID. Return pointer to the frame when
 * found, NULL when not found.
 */
LIN_Frame* FindRxFrame(uint8_t ID);

/*
 * Initialize the timer value variable and start the timer.
 */
void timeoutTimerStart(void);



#endif /* INC_LIN_H_ */
