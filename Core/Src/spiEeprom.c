/*
 * spiEeprom.c
 *
 *  Created on: Sep 9, 2020
 *      Author: Karel Hevessy
 */

#include "spiEeprom.h"
#include "main.h"


/* Buffers for eeprom testing */
uint8_t eeTestRxData[EEPROM_SIZE];
uint8_t eeTestTxData[EEPROM_SIZE];

/* Currently saved / received byte (in the driver) */
uint8_t spiRxByte;
uint8_t spiTxByte;

/* Helper counter for current operation */
volatile int16_t EepromCnt = 0;

/* Counter for waiting to timeout */
volatile uint16_t EepromWCnt = 0;

/* Datalen of the current operation */
int16_t EepromLen;

/* Address of current operation */
uint16_t EepromAdr;

/* Callback function after operation done */
void (*EepromCB)(int16_t);
uint8_t *EepromBuf;

volatile uint8_t EepromMode = 0;

void waitTensOfNs(uint8_t nr)
{
  uint16_t i = 0;
  while (i++ < nr);
}

int8_t EEPROMReadStatus(uint8_t *st, void (*cbfn)(int16_t))
{
  if (EepromMode)
    return -1;
  EepromCB = cbfn;
  EepromBuf = st;
  EepromCnt = -1;
  RESET_EE_CS_PIN();
  EepromMode = EE_READ_ST;
  beginNextByteRxTx(EEPROM_CMD_RDSR);
  return 0;
}

int8_t EEPROMWriteStatus(uint8_t *st, void (*cbfn)(int16_t))
{
  if (EepromMode)
    return -1;
  EepromCB = cbfn;
  EepromBuf = st;
  EepromAdr = 0;
  EepromCnt = -2;
  EepromLen = 0;
  RESET_EE_CS_PIN();
  EepromMode = EE_WRITE_ST;
  waitTensOfNs(EE_CS_SETUP);
  beginNextByteRxTx(EEPROM_CMD_WREN);
  return 0;
}

int8_t EEPROMWait()
{
  while (EepromMode > 0);
  return EepromMode;
}

int8_t EEPROMStat()
{
  return EepromMode;
}

int8_t EEPROMReadData(uint16_t adr, uint8_t *buf, uint16_t len, void (*cbfn)(int16_t))
{
  if (EepromMode > 0)
    return -1;
  EepromCB = cbfn;
  EepromBuf = buf;
  EepromAdr = adr;
  EepromCnt = -3;
  EepromLen = len;
  RESET_EE_CS_PIN();
  EepromMode = EE_READ;
  waitTensOfNs(EE_CS_SETUP);
  beginNextByteRxTx(EEPROM_CMD_READ);
  return 0;
}

int8_t EEPROMWriteData(uint16_t adr, uint8_t *buf, uint16_t len, void (*cbfn)(int16_t))
{
  if (EepromMode > 0)
    return -1;
  EepromCB = cbfn;
  EepromBuf = buf;
  EepromAdr = adr;
  EepromCnt = -3;
  EepromLen = len;
  RESET_EE_CS_PIN();
  EepromMode = EE_WRITE;
  EepromWCnt = 0;
  waitTensOfNs(EE_CS_DISABLE);
  beginNextByteRxTx(EEPROM_CMD_WREN);
  return 0;
}

int8_t EEPROMErase(uint16_t adr, uint16_t len, void (*cbfn)(int16_t))
{
  if (EepromMode > 0)
    return -1;
  EepromCB = cbfn;
  EepromAdr = adr;
  EepromCnt = -3;
  EepromLen = len;
  RESET_EE_CS_PIN();
  EepromMode = EE_ERASE;
  waitTensOfNs(EE_CS_SETUP);
  beginNextByteRxTx(EEPROM_CMD_WREN);
  return 0;
}

void beginNextByteRxTx(uint8_t byte)
{
  spiTxByte = byte;
  LL_SPI_TransmitData8(SPI1, spiTxByte);
}

uint8_t EepromSpiCallback(void)
{
  // RXNE flag cleared after the read
  spiRxByte = LL_SPI_ReceiveData8(SPI1);

  uint8_t sr;

  if (EepromMode == EE_READ)   /* Reading from EEPROM */
  {
    if (EepromCnt == -3)  /* Begin by transmitting the address */
      spiTxByte = (EepromAdr >> 8) & 0xFF;
    else if (EepromCnt == -2)
      spiTxByte = (EepromAdr & 0xFF);
    else if (EepromCnt == -1)   /* Address transfered, we will receive the data */
      spiTxByte = 0;
    else
    {
      spiTxByte = 0;
      EepromBuf[EepromCnt] = spiRxByte;

      if (EepromCnt == EepromLen - 1)   /* Read complete */
      {
        EepromMode = 0;
        waitTensOfNs(EE_CS_HOLD);
        SET_EE_CS_PIN();
        if (EepromCB)
          EepromCB(EepromLen);
        return 0;
      }
      else
        spiTxByte = 0;
    }
    EepromCnt++;
  }
  else if ((EepromMode == EE_WRITE) || (EepromMode == EE_ERASE))  /* Writing to EEPROM */
  {
    if (EepromCnt == -3)
    {
      /* After write enable command, we must cycle the CS pin */
      waitTensOfNs(EE_CS_HOLD);
      SET_EE_CS_PIN();
      spiTxByte = EEPROM_CMD_WRITE;
      waitTensOfNs(EE_CS_DISABLE);
      RESET_EE_CS_PIN();
      waitTensOfNs(EE_CS_SETUP);
    }
    else if (EepromCnt == -2) /* Transfer the address first */
      spiTxByte = (EepromAdr >> 8) & 0xFF;
    else if (EepromCnt == -1)
      spiTxByte = EepromAdr & 0xFF;
    else    /* We can send the data */
    {
      /* Write page ended - new write to the write enable latch needed */
      if ((((EepromCnt + EepromAdr) & (EEPROM_WRITE_BLOCK - 1)) == 0)
          && (EepromCnt > 0))
      {
        /* Cycle of CS pin needed */
        waitTensOfNs(EE_CS_HOLD);
        SET_EE_CS_PIN();
        EepromLen -= EepromCnt;
        EepromAdr += EepromCnt;
        EepromBuf += EepromCnt;
        waitTensOfNs(EE_CS_DISABLE);
        RESET_EE_CS_PIN();
        waitTensOfNs(EE_CS_SETUP);
        spiTxByte = EEPROM_CMD_RDSR;
        if (EepromMode == EE_WRITE)
          EepromMode = EE_WRITE_WAIT;
        else
          EepromMode = EE_ERASE_WAIT;
        EepromWCnt = 0;
      }
      else if (EepromCnt < EepromLen) /* We can transfer more data in this cycle */
      {
        if (EepromMode == EE_WRITE)
          spiTxByte = EepromBuf[EepromCnt];
        else
          spiTxByte = 0xFF;
      }
      else
      {
        waitTensOfNs(EE_CS_HOLD);
        SET_EE_CS_PIN();
        EepromLen = 0;
        waitTensOfNs(EE_CS_DISABLE);
        RESET_EE_CS_PIN();
        waitTensOfNs(EE_CS_SETUP);
        /* Data transfer complete, we can wait for the write to finish and exit */
        spiTxByte = EEPROM_CMD_RDSR;
        if (EepromMode == EE_WRITE)
          EepromMode = EE_WRITE_WAIT;
        else
          EepromMode = EE_ERASE_WAIT;
        EepromWCnt = 0;
      }
    }
    EepromCnt++;
  }
  else if ((EepromMode == EE_WRITE_WAIT) || (EepromMode == EE_ERASE_WAIT))  /* Wait until the write is complete */
  {
    if (EepromWCnt & 1)
    {
      waitTensOfNs(EE_CS_HOLD);
      SET_EE_CS_PIN();
      sr = spiRxByte;
      if (EepromWCnt > EEPROM_TIMEOUT)    /* Timeout of the communication - EEPROM not responding, error */
      {
        EepromMode = -1;
        if (EepromCB)
          EepromCB(-1);
        return 0;
      }
      else if (sr & 1)  /* Write not complete yet */
      {
        waitTensOfNs(EE_CS_DISABLE);
        RESET_EE_CS_PIN();
        waitTensOfNs(EE_CS_SETUP);
        spiTxByte = EEPROM_CMD_RDSR;
      }
      else  /* Data successfully written */
      {
        if (EepromLen)
        {
          waitTensOfNs(EE_CS_DISABLE);
          RESET_EE_CS_PIN();
          waitTensOfNs(EE_CS_SETUP);
          EepromCnt = -3;
          /* New write cycle can begin */
          spiTxByte = EEPROM_CMD_WREN;
          if (EepromMode == EE_WRITE_WAIT)
            EepromMode = EE_WRITE;
          else
            EepromMode = EE_ERASE;
        }
        else
        {
          EepromMode = 0;
          if (EepromCB)
            EepromCB(EepromWCnt);
          return 1;
        }
      }
    }
    else
      spiTxByte = 0;
    EepromWCnt++;
  }
  else if (EepromMode == EE_READ_ST)    /* Read status reg */
  {
    if (EepromCnt == -1)
      spiTxByte = 0;
    else  /* Read complete */
    {
      sr = spiRxByte;
      EepromBuf[0] = sr;
      EepromMode = 0;
      waitTensOfNs(EE_CS_HOLD);
      SET_EE_CS_PIN();
      if (EepromCB)
        EepromCB(sr);
      return 0;
    }
    EepromCnt++;
  }
  else if (EepromMode == EE_WRITE_ST)   /* Write to status reg */
  {
    if (EepromCnt == -2)
    {
      waitTensOfNs(EE_CS_HOLD);
      SET_EE_CS_PIN();
      waitTensOfNs(EE_CS_DISABLE);
      RESET_EE_CS_PIN();
      waitTensOfNs(EE_CS_SETUP);
      spiTxByte = EEPROM_CMD_WRSR;
    }
    else if (EepromCnt == -1)
    {
      spiTxByte = *EepromBuf;   /* Actually write the data */
    }
    else  /* Write of the status register complete */
    {
      waitTensOfNs(EE_CS_HOLD);
      SET_EE_CS_PIN();
      EepromLen = 0;
      waitTensOfNs(EE_CS_DISABLE);
      RESET_EE_CS_PIN();
      waitTensOfNs(EE_CS_SETUP);
      spiTxByte = EEPROM_CMD_RDSR;
      EepromMode = EE_WRITE_WAIT;
      EepromWCnt = 0;
    }
    EepromCnt++;
  }

  /* Begin next SPI transmission */
  LL_SPI_TransmitData8(SPI1, spiTxByte);

  return 0;
}

void TestEepromAccess(void)
{
  /* Init the test tx array */
  for (int i = 0; i < EEPROM_SIZE; i++)
    eeTestTxData[i] = i;

  EEPROMWriteData(0, eeTestTxData, EEPROM_SIZE, NULL);
  EEPROMWait();

  EEPROMReadData(0, eeTestRxData, EEPROM_SIZE, NULL);
  EEPROMWait();

  uint8_t good = 1;

  /* Test the read data */
  for (uint16_t i = 0; i < EEPROM_SIZE; i++)
    if (eeTestRxData[i] != eeTestTxData[i])
    {
      good = 0;
      break;
    }

  /* Try erasing the eeprom */
  EEPROMErase(0, EEPROM_SIZE, NULL);
  EEPROMWait();

  /* Test that all the data is 0xff */
  EEPROMReadData(0, eeTestRxData, EEPROM_SIZE, NULL);
  EEPROMWait();
  for (uint16_t i = 0; i < EEPROM_SIZE; i++)
    if (eeTestRxData[i] != 0xff)
    {
      good = 0;
      break;
    }

  /* If there was some problem, turn on the red LED */
  if (!good)
    LED_R_ON();
  else
    LED_G_ON();
}
