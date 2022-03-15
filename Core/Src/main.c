/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : CANFD LIN Gateway Example
  ******************************************************************************
  * @attention
  *
  * Product: CANFD LIN Gateway
  * Product Number: CANFD-LIN-GW
  *
  * Company: MACH SYSTEMS s.r.o.
  * Company Web: www.machsystems.cz
  *
  * Product Description:
  * - Two high-speed CAN channels with CAN FD support
  * - LIN channel
  * - RS-232 port
  * - MicroSD card slot
  * - 4 digital outputs
  * - 2 analogue/digital inputs
  * - 4 status LEDs
  * - On-board 16 Kbit EEPROM
  * - Externally or USB-powered
  * - Table-top use or DIN-rail mount
  *
  * MCU: STM32G483RE
  * IDE: STM32CubeIDE v1.9.0
  * HAL: STM32CubeG4 Firmware Package V1.5.0
  *
  *
  * Connector pinout of the CANFD LIN Gateway device:
  * =========================
  * D-SUB9 Male:  Pin 1    - Digital Output 1 (HSD)
  *               Pin 2    - CAN1 Low
  *               Pin 3, 5 - GND
  *               Pin 4    - LIN
  *               Pin 6    - Analog input 1 (0 - 5 V)
  *               Pin 7    - CAN1 High
  *               Pin 8    - Digital Output 2 (5 V push-pull)
  *               Pin 9    - Vin / Vbat (7 - 30 V DC)
  *
  * D-SUB9 Female: Pin 1    - Digital Output 3 (5 V push-pull)
  *                Pin 2    - CAN2 Low
  *                Pin 3, 5 - GND
  *                Pin 4    - RS232 RxD (in)
  *                Pin 6    - Digital Output 4 (LSD-Open drain)
  *                Pin 7    - CAN2 High
  *                Pin 8    - RS232 TxD (out)
  *                Pin 9    - Analog Input 2 (0 - 5 V)
  *
  *
  * Description of the example:
  * ===========================
  * SD Card:
  *   If you want to test SD card access, uncomment the line with
  * #define TEST_SDCARD. If you do this, program will try to create
  * new file on the card and write some text to it. Then, it will try
  * to read back the written data and check if it is correct.
  *
  * CAN-FD:
  *   Default setting for CAN is 500k, 2M, SP = 80 %.
  *   After pressing the button BTN 1, CAN message is sent on both
  * CAN1 and CAN2. First data byte is number of sent message, second
  * and third have state of dip switches as LSB. Fourth+fifth and sixth+seventh
  * data bytes are voltages (in millivolts) measured on AI1 and AI2 respectively.
  * Furthemore, data received on CAN are sent to the other CAN channel.
  *   If you want CAN FD data baud rate 8 Mbps, you must change FDCAN clock to
  * 144 MHz and change time segments accordingly.
  * Ratio of data bit rate to nominal bit rate should not exceed 8, or
  * sometimes error frames can be sent.
  *
  * LIN:
  *   In LIN Scheduler, there are 2 frames:
  * ID 0x01 - Master Response
  *         - 3 data bytes
  *         - LSB of bytes 0, 1, 2 contain respectively state of
  *           button, dip 1 and dip 2
  *
  * ID 0x02 - Master Request
  *         - 2 data bytes
  *         - LSB of byte 0: toggle LED1, LSB of byte 1: toggle LED2
  *           - 0x01 turn on
  *           - 0x00 turn off
  *
  * Bootloader:
  *   CAN frame with extended frame ID 0x1fffffff and first four data bytes 0x00
  * 0x01 0x02 0x03 (on CAN1 or CAN2) will reset the device to bootloader. If you
  * then connect USB cable or one of UARTs, you can upload new firmware to the
  * device. See manual for more info.
  *
  * Virtual COM Port:
  *   You can connect to the COM port via USB cable. When CAN frame is received,
  * some info about it is sent to the COM port. There is ID, on which channel it
  * was received, number of data bytes and the data. Also, data received on the
  * COM port are sent to debug UART. For its settings, see below.
  *
  * EEPROM:
  *   Program tries to write the whole EEPROM (2 KiB) with data and then read it
  * back. It checks if the read data is correct. Then, the whole EEPROM is erased,
  * again read back and checked that it contains 0xFF. Uncomment EEPROM_TESTING
  * for activating this example.
  *
  * RS-232:
  *   Received data are sent back. Uncomment TEST_UART for this example.
  * Settings are: 115200, 8 B, 1 stop bit, no parity.
  *
  * Debug UART:
  *   Data received on Virtual COM port are sent to debug UART. Settings are:
  * 115200, 8 B, 1 stop bit, no parity.
  *
  * Digital Inputs:
  *   There are three digital inputs, one button and two dip switches. For reading
  * their value, see CAN section above.
  *
  * Analog Inputs:
  *   There are two analog inputs that can thanks measure 0 to 5 V. For reading their
  * value, see CAN section above.
  *
  * Outputs:
  *   There are four outputs, one HSD (DO1), two 5 V push-pulls (DO2 and DO3) and one open
  * drain (DO4). You can use them freely but please make sure you do NOT your loads do not draw
  * too high current from 5 V power branch especially.
  * DO NOT use DO1, DO2, DO3 when the device is only powered from USB!!!
  * There are macros defined in main.h (in section Digital outputs)
  * for controlling them, but they are not used in this example.
  *
  * Also, there are three LEDs, macros for using them are in main.h in sections
  * User LED controls and CAN LED control.
  *
  *
  *
  * Mapping of peripherals to their names:
  * ======================================
  * CAN1: FDCAN2 peripheral
  * CAN2: FDCAN3 peripheral
  *
  * ADIN1: IN15 of ADC1
  * ADIN2: IN12 of ADC1
  *
  * RS232:      UART1
  * DEBUG UART: UART2
  * LIN:        UART3
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "commonUtils.h"
#include "spiEeprom.h"

#include "lin.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_MAX_DATALEN       64

//#define EEPROM_TESTING
#define LIN_EXAMPLE
//#define TEST_SDCARD
//#define TEST_UART     /* Blocking RS-232 example */

#define LIN_TICKS_PER_SECOND  1000
#define LIN_TICKS_WAIT        10
#define USB_BUFFER_SIZE       512

#define ADC_CONVERSION        (3.3 / 4095)    /* 3.3 V reference 12-bit ADC */
#define DIVIDER               (50.0 / 30)     /* R1 = 20 000 ohm, R2 = 50 000 ohm  */

#define LIN_TX_DATA_SIZE      3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//uint8_t ubKeyNumber = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[CAN_MAX_DATALEN];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[CAN_MAX_DATALEN];
uint8_t canFramesSent = 0;

/* FATFs process status */
int32_t ProcessStatus = 0;

uint16_t gTimerCnt = 0;
uint16_t gPeriodCnt = 0;

/* String to send via uart */
uint8_t aTextInfoStart[] = "\r\nUART Debug Example: Received data will be sent back:\r\n";

/* From usb_device.c */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ADC buffer and state */
uint8_t adcState = 0;
uint32_t adcValues[2];

/* Request to go to bootloader */
uint8_t BootloaderRequest = 0;

/* LIN variables */
uint8_t linTxData[LIN_TX_DATA_SIZE] = {0, 0, 0};
LIN_Frame* pLinRxFrame;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void jumpToBootloader(void);
void initCanTxData(void);
void initAdc(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_FDCAN3_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_TIM6_Init();
  MX_USB_Device_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  initCanTxData();

  PIN_CAN1_STDBY_ON();
  PIN_CAN2_STDBY_ON();
  /* Initialize the micro SD Card */
  BSP_SD_Init(0);
  /* Configure the FDCAN peripherals */
  FdcanConfig(&hfdcan2);
  FdcanConfig(&hfdcan3);
  HAL_TIM_Base_Start_IT(&htim6);
  initAdc();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef LIN_EXAMPLE
  /* Wake up the LIN transceiver */
  HAL_GPIO_WritePin(LIN_CS_GPIO_Port, LIN_CS_Pin, 1);
  HAL_GPIO_WritePin(LIN_WAKE_GPIO_Port, LIN_WAKE_Pin, 0);

  InitLin();

  /* Try to find the Master Request frame with ID 0x2 (for toggling the LEDs) */
  pLinRxFrame = FindRxFrame(2);
#endif /* LIN_EXAMPLE */

  /*
   * Configuration of SPI1 peripheral for EEPROM access
   */
  /* enable error interrupt */
  LL_SPI_EnableIT_ERR(SPI1);

  /* RXNE event after 8 bits */
  LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

  /* enable RXNE interrupt */
  LL_SPI_EnableIT_RXNE(SPI1);

  LL_SPI_Enable(SPI1);

#ifdef EEPROM_TESTING
  TestEepromAccess();
#endif /* EEPROM_TESTING */

  FdcanReconfigureBaudrate(&hfdcan2, CAN_NBT_500K, CAN_DBT_2M);

#ifdef TEST_UART
  /* Do not send ending '\0' */
  HAL_UART_Transmit(&huart1, aTextInfoStart, sizeof(aTextInfoStart) - 1, HAL_MAX_DELAY);
#endif /* TEST_UART */

  while (1)
  {
    if (BootloaderRequest)
      jumpToBootloader();

#ifdef TEST_UART
    /* Uart testing - send back what is received, byte after byte */
    uint8_t receive;
    HAL_UART_Receive(&huart1, &receive, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, &receive, 1, HAL_MAX_DELAY);
#endif /* TEST_UART */

#ifdef TEST_SDCARD
    /* FatFs middleware main background task */
    ProcessStatus = MX_FATFS_Process();

    if (ProcessStatus == APP_SD_UNPLUGGED)
    {
      LED_G_OFF();
      LED_R_TOGGLE();
    }
    else if (ProcessStatus == APP_ERROR)
    {
      Error_Handler();
    }
    else
    {
      LED_R_OFF();
      LED_G_ON();
    }
    HAL_Delay(1000);
#endif /* TEST_SDCARD */

#ifdef LIN_EXAMPLE
    /* Toggle LEDs according to data bytes */
    /* (if the frame was found) */
    if (pLinRxFrame != NULL)
    {
      /* LED 1 */
      if (pLinRxFrame->Data[0] & 0x1)
        LED_G_ON();
      else
        LED_G_OFF();

      /* LED 2 */
      if (pLinRxFrame->Data[1] & 0x1)
        LED_CAN2_Y_ON();
      else
        LED_CAN2_Y_OFF();
    }

    /* Btn active in one (changed from earlier HW) */
    if (RESET_USER_READ())
      linTxData[0] |= 1;
    else
      linTxData[0] &= 0;

    /* Dip switches active in zero */
    if (!DIPSW1_READ())
      linTxData[1] |= 1;
    else
      linTxData[1] &= 0;

    if (!DIPSW2_READ())
      linTxData[2] |= 1;
    else
      linTxData[2] &= 0;

    UpdateTxFrameData(1, linTxData, LIN_TX_DATA_SIZE);
#endif /* LIN_EXAMPLE */


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 8;
  hfdcan2.Init.NominalSyncJumpWidth = 3;
  hfdcan2.Init.NominalTimeSeg1 = 14;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 2;
  hfdcan2.Init.DataSyncJumpWidth = 3;
  hfdcan2.Init.DataTimeSeg1 = 14;
  hfdcan2.Init.DataTimeSeg2 = 3;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 8;
  hfdcan3.Init.NominalSyncJumpWidth = 3;
  hfdcan3.Init.NominalTimeSeg1 = 14;
  hfdcan3.Init.NominalTimeSeg2 = 3;
  hfdcan3.Init.DataPrescaler = 2;
  hfdcan3.Init.DataSyncJumpWidth = 3;
  hfdcan3.Init.DataTimeSeg1 = 14;
  hfdcan3.Init.DataTimeSeg2 = 3;
  hfdcan3.Init.StdFiltersNbr = 0;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 143;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */
  /* Timer 7 will actually be setup in the user code (TIMER7_init); it is
   * because the LIN baudrate */
  /* USER CODE END TIM7_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

  /* TIM7 interrupt Init */
  NVIC_SetPriority(TIM7_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM7_DAC_IRQn);

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM7);
  LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM7);
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**USART3 GPIO Configuration
  PC10   ------> USART3_TX
  PC11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART3 interrupt Init */
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART3_IRQn);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART3);
  LL_USART_SetLINBrkDetectionLen(USART3, LL_USART_LINBREAK_DETECT_10B);
  LL_USART_DisableOverrunDetect(USART3);
  LL_USART_DisableDMADeactOnRxErr(USART3);
  LL_USART_ConfigLINMode(USART3);

  /* USER CODE BEGIN WKUPType USART3 */

  /* USER CODE END WKUPType USART3 */

  LL_USART_Enable(USART3);

  /* Polling USART3 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART3))) || (!(LL_USART_IsActiveFlag_REACK(USART3))))
  {
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_USER_R_Pin|LED_USER_G_Pin|LED_CAN1_R_Pin|LED_CAN1_Y_Pin
                          |OUT_3_OD_Pin|OUT_2_PP_Pin|_5V_OUT_EN_Pin|_5V_OUT_FAULT_Pin
                          |LIN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_CAN2_R_Pin|LED_CAN2_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_MEM_CS_GPIO_Port, SPI_MEM_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_1_PP_Pin|LIN_WAKE_Pin|LIN_MASTER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN2_STB_GPIO_Port, CAN2_STB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN1_STB_GPIO_Port, CAN1_STB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DIPSW_1_Pin DIPSW_2_Pin */
  GPIO_InitStruct.Pin = DIPSW_1_Pin|DIPSW_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_USER_R_Pin LED_USER_G_Pin LED_CAN1_R_Pin LED_CAN1_Y_Pin
                           OUT_2_PP_Pin _5V_OUT_EN_Pin _5V_OUT_FAULT_Pin CAN2_STB_Pin
                           LIN_CS_Pin */
  GPIO_InitStruct.Pin = LED_USER_R_Pin|LED_USER_G_Pin|LED_CAN1_R_Pin|LED_CAN1_Y_Pin
                          |OUT_2_PP_Pin|_5V_OUT_EN_Pin|_5V_OUT_FAULT_Pin|CAN2_STB_Pin
                          |LIN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_CAN2_R_Pin LED_CAN2_Y_Pin */
  GPIO_InitStruct.Pin = LED_CAN2_R_Pin|LED_CAN2_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_MEM_CS_Pin */
  GPIO_InitStruct.Pin = SPI_MEM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_MEM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_3_OD_Pin */
  GPIO_InitStruct.Pin = OUT_3_OD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_3_OD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_1_PP_Pin LIN_WAKE_Pin */
  GPIO_InitStruct.Pin = OUT_1_PP_Pin|LIN_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN1_STB_Pin */
  GPIO_InitStruct.Pin = CAN1_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN1_STB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_USER_Pin */
  GPIO_InitStruct.Pin = RESET_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RESET_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIN_MASTER_Pin */
  GPIO_InitStruct.Pin = LIN_MASTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIN_MASTER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/*
 * General purpose 1 ms timer callback
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    gTimerCnt++;
    if (gTimerCnt == 10)
    {
      HAL_ADC_Start_IT(&hadc1);
      gTimerCnt = 0;
    }

    /* LIN scheduler timer tick */
    SchedulerTimerCallback();
  }
}

/*
 * ADC conversion complete callback
 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  double val = HAL_ADC_GetValue(hadc);
  val = (val * ADC_CONVERSION) * DIVIDER;
  val *= 1000; /* To millivolts */
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  switch (adcState)
  {
    case 0:
      adcValues[0] = val;
      sConfig.Channel = ADC_CHANNEL_12;
      adcState++;
      break;

    case 1:
    default:
      adcValues[1] = val;
      sConfig.Channel = ADC_CHANNEL_15;
      adcState = 0;
      break;
  }
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  HAL_ADC_Stop_IT(&hadc1);
}


/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      Error_Handler();
    }
    uint8_t datalen = DatalenToInt(RxHeader.DataLength);

    /* It is strongly suggested to keep the possibility to jump to System Booloader from application */
    if (RxHeader.Identifier == 0x1fffffff && datalen == 4 && RxData[0] == 0 && RxData[1] == 1 && RxData[2] == 2 && RxData[3] == 3)
    {
      /* Cannot go to bootloader directly from ISR */
      BootloaderRequest = 1;
      return;
    }

    /* Send the information about received CAN frame to the virtual COM port */
    char usbSendBuffer[USB_BUFFER_SIZE];
    uint8_t dataWritten = 0;
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE, "ID: %lx, ", RxHeader.Identifier);
    char* chan = (hfdcan == &hfdcan2) ? "CAN1" : "CAN2";
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE, "channel: %s, ", chan);
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE, "datalen: %d, ", datalen);
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE, "data: ");
    for (uint8_t i = 0; i < datalen; i++)
    {
      dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE, "%x", RxData[i]);
      if (i != datalen - 1)
        dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE, ", ");
    }
    dataWritten += snprintf(usbSendBuffer + dataWritten, USB_BUFFER_SIZE, "\r\n");
    CDC_Transmit_FS((uint8_t*) usbSendBuffer, dataWritten);

    /* Respond back with the same data and header */
    FDCAN_HandleTypeDef sendCan = (hfdcan == &hfdcan2) ? hfdcan3 : hfdcan2;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&sendCan, (FDCAN_TxHeaderTypeDef*) &RxHeader, RxData) != HAL_OK)
      Error_Handler();

//    /* Display LEDx */
//    if ((RxHeader.Identifier == 0x321) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_2))
//    {
//      LED_Display(RxData[0]);
//      ubKeyNumber = RxData[0];
//    }
  }

}

/*
 * Error callback from HAL - reset the peripheral
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  DeviceCANErrorHandler(hfdcan);
}

/*
 * Error callback from HAL - reset the peripheral
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  DeviceCANErrorHandler(hfdcan);
}

/*
 * Initialize the data and header that will be sent to CAN
 */
void initCanTxData(void)
{
  /* Prepare Tx Header */
  TxHeader.Identifier = 0x321;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_3;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  /* Next two lines would make the frame FD CAN with BRS on */
  /*TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN;*/
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  for (uint8_t i = 0; i < CAN_MAX_DATALEN; i++)
    TxData[i] = i;
}

void initAdc(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  HAL_ADC_Start_IT(&hadc1);
}

/*
 * Called from interrupt when some data from USB is received
 */
void UsbDataReceived(uint8_t* pBuffer, uint16_t size)
{
  /* Send the received data right back */
  //CDC_Transmit_FS(pBuffer, size);

  /* Send the data to the debug UART */
  HAL_UART_Transmit(&huart2, pBuffer, size, HAL_MAX_DELAY);
}

/**
 * Function to perform jump to system memory boot from user application.
 *
 * Call function when you want to jump to system memory.
 * Inspired by example code from:
 *     https://stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32/
 */
void jumpToBootloader(void)
{
    void (*SysMemBootJump)(void);

    __disable_irq();
    /* Reset all configured peripherals */
    HAL_DeInit();
    // deconfigure the usb
    //USBD_DeInit(&hUsbDeviceFS);
    /* System clock frequency 72 MHz - PLL clocked by HSI */
    HAL_RCC_DeInit();

    /* USB must be clocked by HSI48 @ 48 MHz */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /**
     * Step: Set system memory address.
     *
     *       For STM32G4, system memory is on 0x1FFF 0000
     */
    volatile uint32_t addr = 0x1FFF0000;

    /**
     * Step: Disable RCC, set it to default (after reset) settings
     *       Internal clock, no PLL, etc.
     */
    HAL_RCC_DeInit();

    /**
     * Step: Disable systick timer and reset it to default values
     */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /* ARM Cortex-M Programming Guide to Memory Barrier Instructions.*/
    __DSB();

    /**
     * Step: Remap system memory to address 0x0000 0000 in address space
     * Call HAL macro to do this
     */
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    /* Remap is not visible at once. Execute some unrelated command! */
    __DSB();
    __ISB();

    /* Clear Interrupt Enable Register & Interrupt Pending Register */
    for (int i = 0; i < 5; i++)
    {
      NVIC->ICER[i]=0xFFFFFFFF;
      NVIC->ICPR[i]=0xFFFFFFFF;
    }

    SCB->VTOR = 0;                /* Set vector table offset to 0 */

    /* Re-enable all interrupts */
    /* Cannot leave interrupts disabled - no one would enable them */
    __enable_irq();

    /**
     * Step: Set jump memory location for system memory
     *       Use address with 4 bytes offset which specifies jump location where program starts
     */
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

    /**
     * Step: Set main stack pointer.
     *       This step must be done last otherwise local variables in this function
     *       don't have proper value since stack pointer is located on different position
     *
     *       Set direct address location which specifies stack pointer in SRAM location
     */
    __set_MSP(*(__IO uint32_t *)addr);

    /**
     * Step: Actually call our function to jump to set location
     *       This will start system memory execution
     */
    SysMemBootJump();

    /* Now we are in bootloader */
}

/*
 * SPI1 callback for EEPROM access
 */
void SpiInterruptCallback(void)
{
  EepromSpiCallback();
}

/*
 * External interrupt callback - user button was pressed.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_8)
  {
    TxData[0] = ++canFramesSent;
    TxData[1] = (!DIPSW1_READ()) & 0x1;
    TxData[2] = (!DIPSW2_READ()) & 0x1;

    /* IN1 */
    TxData[3] = adcValues[0] & 0xff;
    TxData[4] = (adcValues[0] >> 8) & 0xff;
    /* IN2 */
    TxData[5] = adcValues[1] & 0xff;
    TxData[6] = (adcValues[1] >> 8) & 0xff;

    TxHeader.DataLength = FDCAN_DLC_BYTES_7;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, (FDCAN_TxHeaderTypeDef*) &TxHeader, TxData);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, (FDCAN_TxHeaderTypeDef*) &TxHeader, TxData);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* If there is error on one of the used CAN peripherals, reset it */
  uint32_t error1 = HAL_FDCAN_GetError(&hfdcan2);
  uint32_t error2 = HAL_FDCAN_GetError(&hfdcan3);

  if (error1)
    DeviceCANErrorHandler(&hfdcan2);
  if (error2)
    DeviceCANErrorHandler(&hfdcan3);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
