/*
 * spiEeprom.h
 *
 *  Created on: Sep 9, 2020
 *      Author: Karel Hevessy
 *
 *
 *  EEPROM SPI driver.
 *  All functions have parameter cbfn, which is callback function that is called after completion
 *  of the operation.
 */

#ifndef INC_SPIEEPROM_H_
#define INC_SPIEEPROM_H_

#include "stdint.h"

#define EEPROM_CMD_READ     (uint8_t) 0b00000011
#define EEPROM_CMD_WRITE    (uint8_t) 0b00000010
#define EEPROM_CMD_WRDI     (uint8_t) 0b00000100
#define EEPROM_CMD_WREN     (uint8_t) 0b00000110
#define EEPROM_CMD_RDSR     (uint8_t) 0b00000101
#define EEPROM_CMD_WRSR     (uint8_t) 0b00000001

/* 2 ms timeout */
#define EEPROM_TIMEOUT        2250000/500

/* Pages are 16 B */
#define EEPROM_WRITE_BLOCK    16

/* Instruction set of the EEPROM */
#define EE_READ               1
#define EE_WRITE              2
#define EE_WRITE_WAIT         3
#define EE_READ_ST            4
#define EE_WRITE_ST           5
#define EE_ERASE              6
#define EE_ERASE_WAIT         7

/* Minimal CS pin timings - values for Vdd 3.3 V */
#define EE_CS_SETUP           1   /* CS low before the transaction - 100 ns*/
#define EE_CS_HOLD            2   /* CS low after the transaction - 200 ns*/
#define EE_CS_DISABLE         1   /* CS must stay high for at least 100 ns */

/* Buffer size for EEPROM testing; same as size of the whole EEPROM */
#define EEPROM_SIZE           2048

/*
 * Read of status register
 */
int8_t EEPROMReadStatus(uint8_t* st, void (*cbfn)(int16_t));

/*
 * Write of EEPROM status register
 */
int8_t EEPROMWriteStatus(uint8_t* st, void (*cbfn)(int16_t));

/*
 * Read of block of data of length len
 */
int8_t EEPROMReadData(uint16_t adr, uint8_t* buf, uint16_t len, void (*cbfn)(int16_t));

/*
 * Write of block of length len
 */
int8_t EEPROMWriteData(uint16_t adr, uint8_t* buf, uint16_t len, void (*cbfn)(int16_t));

/*
 * Erase (write of 0xff) of data of length len.
 */
int8_t EEPROMErase(uint16_t adr, uint16_t len, void (*cbfn)(int16_t));

/*
 * Wait for end of current operation.
 */
int8_t EEPROMWait();

/*
 * State of the last operation:
 *   positive number: operation in progress
 *   0              : operation complete
 *   -1             : operation ended by error
 */
int8_t EEPROMStat();

/*
 * Return 1 when the current operation is done, else 0
 */
uint8_t EepromSpiCallback(void);

/*
 * Begin SPI transmission / reception of the supplied byte
 */
void beginNextByteRxTx(uint8_t byte);

/*
 * Wait for very approximately nr tens of nanoseconds
 */
void waitTensOfNs(uint8_t nr);

/*
 * Test writing and reading some data from eeprom
 */
void TestEepromAccess(void);



#endif /* INC_SPIEEPROM_H_ */
