/*
 * lin.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Karel Hevessy
 */

#include <string.h>  /* memset(), memcpy() */
#include "lin.h"
#include "lin_timer.h"
#include "lin_uart.h"

/* Current scheduler delay */
uint16_t LinDelay;

uint8_t linIsTransmitting;

uint8_t schedulerRunning;
uint8_t schedulerIndex;

LinScheduler scheduler;

/* Frame to send */
LIN_Frame txFrame;
uint8_t txFrameDataIndex;
/* Data to receive */
uint8_t rxData[RX_BUFFER_LENGTH];
uint8_t rxDataIndex;

/* What part of the LIN message we will be transmitting */
LinState linTransmitState;

/* Determines if we will transmit anything or will be only receiveing */
uint8_t linStopTx;

/* One if we are receiving the slave response */
uint8_t linReceiving;

/* Response timeout counter variables */
uint16_t responseTimeoutValue;
uint16_t responseTimeoutMax;

/* Timeout of receiving break (in case of short of LIN to Vbat) */
uint8_t breakTmrRunning;
uint16_t breakTmrVal;

void linErrorCallback(void)
{

}

void SchedulerTimerCallback(void)
{
  if (schedulerRunning)
  {
    LinDelay--;
    if (LinDelay == 0)
    {
      if (LinTransmitFrameAsync(&scheduler.Row[schedulerIndex].Frame))
      {
        schedulerIndex = (schedulerIndex + 1) % scheduler.NumberOfFrames;
      }
      LinDelay = scheduler.Row[schedulerIndex].Delay;
    }
  }
}

void LinSchedulerStart(void)
{
  if (scheduler.NumberOfFrames)
  {
    linIsTransmitting = 0;
    schedulerIndex = 0;
    schedulerRunning = 1;
    LinDelay = scheduler.Row[schedulerIndex].Delay;
  }
}

void LinSchedulerStop(void)
{
  schedulerRunning = 0;
}


void InitLin(void)
{
#ifdef LIN_MASTER_MODE
  /* Some Master specific configuration could go here */
#else
  #ifdef LIN_SLAVE_MODE
    #error Lin slave not implemented.
  #else
    #error You must define if you are LIN Master or Slave.
  #endif
#endif

  InitLinUartBaudrate();
  /* Configure the UART interrupts */
  LinUartEnableBreakInterrupt();
  LinUartEnableRxInterrupt();
  LinUartDisableTxInterrupt();

  initLinScheduler();

  resetStateMachine();

  LinSchedulerStart();
}

void initLinScheduler(void)
{
  scheduler.Row[0].Frame.ID = 0x1;
  scheduler.Row[0].Frame.Length = 3;
  memset((void*) scheduler.Row[0].Frame.Data, 0, scheduler.Row[0].Frame.Length);
  scheduler.Row[0].Frame.ChecksumType = ENHANCED_CHECKSUM;
  scheduler.Row[0].Frame.Type = MASTER_RESPONSE;
  scheduler.Row[0].Delay = 25;

  scheduler.Row[1].Frame.ID = 0x2;
  scheduler.Row[1].Frame.ResponseLength = 2;
  scheduler.Row[1].Frame.ChecksumType = ENHANCED_CHECKSUM;
  scheduler.Row[1].Frame.Type = MASTER_REQUEST;
  scheduler.Row[1].Delay = 50;

  scheduler.NumberOfFrames = 2;
}

uint8_t LinTransmitFrameAsync(LIN_Frame* pFrame)
{
  uint8_t ret;
  if (!linIsTransmitting && !linReceiving)
  {
    /* We can begin sending the data */
    txFrame.ID = getParityIdentifier(pFrame->ID);
    if (pFrame->Type == MASTER_RESPONSE)
      memcpy((void*) txFrame.Data, pFrame->Data, pFrame->Length);
    if (pFrame->Type == MASTER_RESPONSE)
      txFrame.Length = pFrame->Length;
    else
      txFrame.ResponseLength = pFrame->ResponseLength;
    txFrame.Length = pFrame->Length;
    txFrame.Type = pFrame->Type;
    txFrame.ChecksumType = pFrame->ChecksumType;
    txFrame.Checksum = calcChecksum(pFrame->Data, pFrame->Length,
                                    pFrame->ChecksumType, pFrame->ID);

    /* Reset the index for receiving the data */
    rxDataIndex = 0;
    linTransmitState = STATE_SYNC_BREAK;
    linIsTransmitting = 1;  /* Transmission in progress */
    linStopTx = 0;
    LinUartEnableTxInterrupt();
    ret = 1;
  }
  return ret;
}

void LinTimeoutTimerCallback(void)
{
  if (breakTmrRunning)
  {
      /* Waiting for break - in case when LIN is shorted to Vbat */
      breakTmrVal++;
      if (breakTmrVal >= BREAK_TIMEOUT_MAX)
      {
          LinTimeoutTimerStop();
          breakTmrVal = breakTmrRunning = 0;
          linResetAll();
      }
  }
  else if (linReceiving) /* Timeout for response running */
  {
    if (responseTimeoutValue < responseTimeoutMax)
      responseTimeoutValue++;
    else
    {
      linReceiving = 0;
      linErrorCallback();
      LinTimeoutTimerStop();
      resetStateMachine();  /* So as not to hang on linReceiving == 1 */
    }
  }
}

void LinRxCallback(uint8_t byteReceived)
{
  rxData[rxDataIndex++] = byteReceived;
  rxDataIndex %= RX_BUFFER_LENGTH;

  if (txFrame.Type == MASTER_REQUEST && linReceiving)
  {
    /* Last byte received */
    if (rxDataIndex >= txFrame.ResponseLength + 1 + 3)
    {
      for (uint8_t i = 0; i < txFrame.ResponseLength + 1 && i < RX_BUFFER_LENGTH - 3; i++)
        rxData[i] = rxData[i + 3];

      linReceiving = 0;
      LinTimeoutTimerStop();
      uint8_t chkTmp;
      if (rxData[txFrame.ResponseLength] == (chkTmp = calcChecksum(rxData, txFrame.ResponseLength,
                                                         txFrame.ChecksumType, txFrame.ID)))
      {
        LIN_Frame* pFr = FindRxFrame(txFrame.ID & 0x3f);
        /* Copy the data to the scheduler frame buffer */
        if (pFr != NULL)
          memcpy(pFr->Data, rxData, txFrame.ResponseLength);
      }
      else
      {
        /* Bad checksum */
        linErrorCallback();
      }
    }
  }
}

void LinTxCallback(void)
{
  if (linStopTx) /* Stop transmitting (done or waiting for the response) */
  {
    if (txFrame.Type == MASTER_REQUEST)
    {
      linReceiving = 1;
      linIsTransmitting = 0;
      responseTimeoutMax = linCalcResponseTimeout(txFrame.ResponseLength);
      timeoutTimerStart();
      /* Disable the tx interrupt */
      LinUartDisableTxInterrupt();
    }
    else
    {
      linIsTransmitting = linReceiving = 0;
      /* Disable the tx interrupt */
      LinUartDisableTxInterrupt();
    }
  }
  else /* Continue with the transmission */
  {
    switch (linTransmitState)
    {
      case STATE_SYNC_BREAK:
        LinUartSendBreak();
        linTransmitState = STATE_SYNC_FIELD;
        /* Trigger timer waiting for break (for the situation when LIN is shorted to Vcc) */
        breakTmrRunning = 1;
        breakTmrVal = 0;
        timeoutTimerStart();
        /* Disable the tx interrupt until the break appears */
        LinUartDisableTxInterrupt();
        break;

      case STATE_ID:
        if (txFrame.Type == MASTER_RESPONSE)
        {
          if (txFrame.Length != 0)
          {
            linTransmitState = STATE_DATA;
            txFrameDataIndex = 0;
          }
          else
            linTransmitState = STATE_CHECKSUM;
        }
        else if (txFrame.Type == MASTER_REQUEST)
        {
          linTransmitState = STATE_SYNC_BREAK;
          /* Do not transmit anything else, wait for response */
          linStopTx = 1;
        }
        else
          linResetAll();

        /* Continue only if reset did not happen */
        if (txFrame.Type == MASTER_REQUEST || txFrame.Type == MASTER_RESPONSE)
          LinUartPutChar(txFrame.ID);
        break;

      case STATE_DATA:
        if (txFrameDataIndex + 1 >= txFrame.Length)
        {
          /* Last data byte will be transmitted, then the checksum */
          linTransmitState = STATE_CHECKSUM;
        }
        LinUartPutChar(txFrame.Data[txFrameDataIndex++]);
        break;

      case STATE_CHECKSUM:
        linTransmitState = STATE_SYNC_BREAK;
        linStopTx = 1;  /* Last byte will be sent */
        LinUartPutChar(txFrame.Checksum);
        break;

      default: /* Bad state */
        linResetAll();
        break;
    }
  }
}

void LinLineBreakCallback(void)
{
  if (linTransmitState == STATE_SYNC_FIELD)
  {
    /* Enable the tx interrupt again */
    LinUartEnableTxInterrupt();
    linTransmitState = STATE_ID;
    LinUartPutChar(0x55);
  }
  else
  {
    linResetAll();
  }
}

void linResetAll(void)
{
  LinTimeoutTimerStop();
  LinUartReset();
  resetStateMachine();
}

void resetStateMachine(void)
{
  linStopTx = linIsTransmitting = linReceiving = 0;
  rxDataIndex = 0;
  linTransmitState = STATE_SYNC_BREAK;
}

void LinFramingErrorCallback(void)
{
  if (linTransmitState == STATE_SYNC_FIELD)
  {
    /* Enable the tx interrupt again */
    LinUartEnableTxInterrupt();
    linTransmitState = STATE_ID;
    LinUartPutChar(0x55);
  }
  else
  {
    /* Line break detected when not wanted - error */
    linResetAll();
  }
}

uint8_t getParityIdentifier(uint8_t id)
{
   uint8_t newId = id & 0x3Fu;
   newId |= (  (id >> 0u & 0x1u) ^ (id >> 1u & 0x1u)
             ^ (id >> 2u & 0x1u) ^ (id >> 4u & 0x1u)) << 6u;
   newId |= (  (id >> 1u & 0x1u) ^ (id >> 3u & 0x1u)
             ^ (id >> 4u & 0x1u) ^ (id >> 5u & 0x1u) ^ 0x1u) << 7u;
   return newId;
}

uint8_t calcChecksum(uint8_t address[], uint8_t size, uint8_t checksumType,
                     uint8_t linId)
{
  uint16_t sum = 0x0000;
  uint8_t i = 0;
  if (ENHANCED_CHECKSUM == checksumType)
  {
    sum += getParityIdentifier(linId);
  }

  for (i = 0; i < size; i++)
    sum += address[i];

  sum = (sum & 0x00FFu) + ((sum & 0xFF00u) >> 8u);
  if (sum & 0xFF00u) /* Did adding the carry bits result in a carry? */
  {
    sum += 1;  /* Add the last carry */
  }

   sum &= 0x00FFu;
   return (uint8_t)(~sum);
}

uint16_t linCalcResponseTimeout(uint8_t numberOfBytes)
{
  /* MaxResponseTimeoutValue is in Tbits. */
  return (numberOfBytes + 1) * 14;
}

uint8_t UpdateTxFrameData(uint8_t ID, uint8_t* pData, uint8_t size)
{
  uint8_t ret = 0;
  /* Prevent buffer overflow */
  if (size > LIN_DATA_LENGTH)
    size = LIN_DATA_LENGTH;

  /* Try to find the frame and update it */
  for (uint8_t i = 0; i < scheduler.NumberOfFrames && i < MAX_SCHEDULER_LENGTH; i++)
  {
    if (scheduler.Row[i].Frame.ID == ID && scheduler.Row[i].Frame.Type == MASTER_RESPONSE)
    {
      memcpy(scheduler.Row[i].Frame.Data, pData, size);
      scheduler.Row[i].Frame.Length = size;
      ret = 1;
      break;
    }
  }
  return ret;
}

LIN_Frame* FindRxFrame(uint8_t ID)
{
  LIN_Frame* ret = NULL;
  /* Try to find the frame */
  for (uint8_t i = 0; i < scheduler.NumberOfFrames && i < MAX_SCHEDULER_LENGTH; i++)
  {
    if (scheduler.Row[i].Frame.ID == ID && scheduler.Row[i].Frame.Type == MASTER_REQUEST)
    {
      ret = &scheduler.Row[i].Frame;
      break;
    }
  }
  return ret;
}


void timeoutTimerStart(void)
{
  responseTimeoutValue = 0;

  /* Reset the counter value */
  LinTimeoutTimerStart();
}
