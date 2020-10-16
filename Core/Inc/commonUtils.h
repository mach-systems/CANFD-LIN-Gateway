/*
 * commonUtils.h
 *
 *  Created on: Sep 8, 2020
 *      Author: Karel Hevessy
 */

#ifndef __COMMON_UTILS_H__
#define __COMMON_UTILS_H__

#include "stm32g4xx_hal.h"

/*
 * CAN bit time setup
 */
typedef enum {
    CAN_500K_500K,
    CAN_500K_1M,
    CAN_500K_2M,
    CAN_500K_3M,
    CAN_500K_4M,
    CAN_500K_5M,
    CAN_500K_6M7,
    CAN_500K_8M,
    CAN_500K_10M,
    CAN_250K_500K,
    CAN_250K_833K,
    CAN_250K_1M,
    CAN_250K_1M5,
    CAN_250K_2M,
    CAN_250K_3M,
    CAN_250K_4M,
    CAN_250K_8M,
    CAN_1000K_1M,
    CAN_1000K_2M,
    CAN_1000K_3M,
    CAN_1000K_4M,
    CAN_1000K_8M,
    CAN_125K_500K,
    CAN_125K_1M,
    CAN_125K_2M,
    CAN_125K_3M,
    CAN_125K_4M,
    CAN_125K_8M
} CAN_BITTIME_SETUP;

/*
 * CAN nominal bit time setup
 */
typedef enum {
    CAN_NBT_125K,
    CAN_NBT_250K,
    CAN_NBT_500K,
    CAN_NBT_1M
} CAN_NOMINAL_BITTIME_SETUP;


/*
 * CAN data bit time setup
 */
typedef enum {
    CAN_DBT_500K,
    //CAN_DBT_833K,
    CAN_DBT_1M,
    //CAN_DBT_1M5,
    CAN_DBT_2M,
    //CAN_DBT_3M,
    CAN_DBT_4M,
    //CAN_DBT_5M,
    //CAN_DBT_6M7,
    CAN_DBT_8M,
    //CAN_DBT_10M
} CAN_DATA_BITTIME_SETUP;

/*
 * Change baudrate of the FDCAN peripheral.
 * Should be called AFTER the initialization.
 */
uint8_t FdcanReconfigureBaudrate(FDCAN_HandleTypeDef* hfdcan, CAN_NOMINAL_BITTIME_SETUP nominalBittime, CAN_DATA_BITTIME_SETUP dataBitTime);

/*
 * Get nominal + data Prescaler, Sync Jump Width and Time Segments
 */
void getCanTimingConstants(FDCAN_HandleTypeDef* pHfdcan, CAN_NOMINAL_BITTIME_SETUP nominalBittime, CAN_DATA_BITTIME_SETUP dataBitTime);

/**
  * @brief  Configures the FDCAN.
  * @param  None
  * @retval None
  */
void FdcanConfig(FDCAN_HandleTypeDef* pFdcan);

/*
 * Reset FDCAN device in case of error
 */
void DeviceCANErrorHandler(FDCAN_HandleTypeDef* pHfdcan);

/*
 * Convert HAL data length to normal number.
 */
uint8_t DatalenToInt(uint32_t datalen);



#endif /* __COMMON_UTILS_H__ */
