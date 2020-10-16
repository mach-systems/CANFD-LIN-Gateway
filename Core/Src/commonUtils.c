/*
 * commonUtils.c
 *
 *  Created on: Sep 8, 2020
 *      Author: Karel Hevessy
 */

#include "commonUtils.h"
#include "main.h"

uint8_t FdcanReconfigureBaudrate(FDCAN_HandleTypeDef* pHfdcan, CAN_NOMINAL_BITTIME_SETUP nominalBittime, CAN_DATA_BITTIME_SETUP dataBitTime)
{
  HAL_FDCAN_DeInit(pHfdcan);

  //hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  //hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  //hfdcan2.Init.AutoRetransmission = DISABLE;
  //hfdcan2.Init.TransmitPause = DISABLE;

  getCanTimingConstants(pHfdcan, nominalBittime, dataBitTime);

  //hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(pHfdcan) != HAL_OK)
    Error_Handler();

  FdcanConfig(pHfdcan);

  return 1;
}

void getCanTimingConstants(FDCAN_HandleTypeDef* pHfdcan, CAN_NOMINAL_BITTIME_SETUP nominalBittime, CAN_DATA_BITTIME_SETUP dataBitTime)
{
  // sample point for all is 80 %
  // 72 MHz
  switch (nominalBittime)
  {
    // 125k
    case CAN_NBT_125K:
      pHfdcan->Init.NominalPrescaler = 16;
      pHfdcan->Init.NominalSyncJumpWidth = 7;
      pHfdcan->Init.NominalTimeSeg1 = 28;
      pHfdcan->Init.NominalTimeSeg2 = 7;

      break;

    // 250k
    case CAN_NBT_250K:
      pHfdcan->Init.NominalPrescaler = 8;
      pHfdcan->Init.NominalSyncJumpWidth = 7;
      pHfdcan->Init.NominalTimeSeg1 = 28;
      pHfdcan->Init.NominalTimeSeg2 = 7;

      break;

    // 500k
    case CAN_NBT_500K:
      pHfdcan->Init.NominalPrescaler = 4;
      pHfdcan->Init.NominalSyncJumpWidth = 7;
      pHfdcan->Init.NominalTimeSeg1 = 28;
      pHfdcan->Init.NominalTimeSeg2 = 7;
      break;

    // 1M
    case CAN_NBT_1M:
      pHfdcan->Init.NominalPrescaler = 2;
      pHfdcan->Init.NominalSyncJumpWidth = 7;
      pHfdcan->Init.NominalTimeSeg1 = 28;
      pHfdcan->Init.NominalTimeSeg2 = 7;

      break;
  }

  switch (dataBitTime)
  {
    // 500k
    case CAN_DBT_500K:
      pHfdcan->Init.DataPrescaler = 4;
      pHfdcan->Init.DataSyncJumpWidth = 7;
      pHfdcan->Init.DataTimeSeg1 = 28;
      pHfdcan->Init.DataTimeSeg2 = 7;
      break;

    // 1M
    case CAN_DBT_1M:
      pHfdcan->Init.DataPrescaler = 2;
      pHfdcan->Init.DataSyncJumpWidth = 7;
      pHfdcan->Init.DataTimeSeg1 = 28;
      pHfdcan->Init.DataTimeSeg2 = 7;
      break;

    // 2M
    case CAN_DBT_2M:
      pHfdcan->Init.DataPrescaler = 1;
      pHfdcan->Init.DataSyncJumpWidth = 7;
      pHfdcan->Init.DataTimeSeg1 = 28;
      pHfdcan->Init.DataTimeSeg2 = 7;

      break;

    // 4M
    case CAN_DBT_4M:
      pHfdcan->Init.DataPrescaler = 1;
      pHfdcan->Init.DataSyncJumpWidth = 3;
      pHfdcan->Init.DataTimeSeg1 = 14;
      pHfdcan->Init.DataTimeSeg2 = 3;
      break;

    // 8M
    case CAN_DBT_8M:
      pHfdcan->Init.DataPrescaler = 1;
      pHfdcan->Init.DataSyncJumpWidth = 2;
      pHfdcan->Init.DataTimeSeg1 = 7;
      pHfdcan->Init.DataTimeSeg2 = 2;

      // values for 144 MHz clock
//      pHfdcan->Init.DataPrescaler = 1;
//      pHfdcan->Init.DataSyncJumpWidth = 3;
//      pHfdcan->Init.DataTimeSeg1 = 14;
//      pHfdcan->Init.DataTimeSeg2 = 3;

      break;
  }
}

void FdcanConfig(FDCAN_HandleTypeDef* pFdcan)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x321;
  sFilterConfig.FilterID2 = 0x7FF;
  if (HAL_FDCAN_ConfigFilter(pFdcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
//  if (HAL_FDCAN_ConfigGlobalFilter(pFdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  /* Activate Rx FIFO 0 new message notification */
//  if (HAL_FDCAN_ActivateNotification(pFdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(pFdcan) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(pFdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }


}

void DeviceCANErrorHandler(FDCAN_HandleTypeDef* pHfdcan)
{
  HAL_FDCAN_Stop(pHfdcan);
  HAL_FDCAN_Start(pHfdcan);
}

uint8_t DatalenToInt(uint32_t datalen)
{
  uint8_t ret;
  switch (datalen)
  {
    case FDCAN_DLC_BYTES_0:
      ret = 0;
      break;
    case FDCAN_DLC_BYTES_1:
      ret = 1;
      break;
    case FDCAN_DLC_BYTES_2:
      ret = 2;
      break;
    case FDCAN_DLC_BYTES_3:
      ret = 3;
      break;
    case FDCAN_DLC_BYTES_4:
      ret = 4;
      break;
    case FDCAN_DLC_BYTES_5:
      ret = 5;
      break;
    case FDCAN_DLC_BYTES_6:
      ret = 6;
      break;
    case FDCAN_DLC_BYTES_7:
      ret = 7;
      break;
    case FDCAN_DLC_BYTES_8:
      ret = 8;
      break;
    case FDCAN_DLC_BYTES_12:
      ret = 12;
      break;
    case FDCAN_DLC_BYTES_16:
      ret = 16;
      break;
    case FDCAN_DLC_BYTES_20:
      ret = 20;
      break;
    case FDCAN_DLC_BYTES_24:
      ret = 24;
      break;
    case FDCAN_DLC_BYTES_32:
      ret = 32;
      break;
    case FDCAN_DLC_BYTES_48:
      ret = 48;
      break;
    case FDCAN_DLC_BYTES_64:
      ret = 64;
      break;
    default:
      ret = 0;
      break;
  }
  return ret;
}
