/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "radio_driver.h"
#include "stm32wlxx_nucleo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief Describes whether this module is the sender or the recipient
 * of the data in this stream. Should be specified on a configuration/hardware
 * level. (TODO)
 */
typedef enum state_t 
{
  STATE_NULL,
  STATE_SENDER,
  STATE_RECIPIENT 
} state_t;

/**
 * @brief Describes what I/O state the LoRa module is
 * set to. Should be set by callbacks.
 */
typedef enum substate_t 
{
  SSTATE_NULL,
  SSTATE_RX,
  SSTATE_TX
} substate_t;

/**
 * @brief Context wrapper for LoRa RX/TX buffer.
 * @note DO NOT MODIFY THE BUFFER SIZE, IT IS WRITTEN
 * ON A HARDWARE-LEVEL AND USELESS PADDING WILL NOT
 * BE WRITTEN TO, AND MAY RESULT IN UNDEFINED BEHAVIOR.
 */
typedef struct LoRaHandle
{
  state_t state;
  substate_t subState;
  uint32_t rxTimeout;
  uint32_t rxMargin;
  uint32_t randomDelay;
  char rxBuffer[RX_BUFFER_SIZE];
  uint8_t rxSize;
} LoRaHandle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define RF_FREQUENCY                                868000000 /* Hz */
#define TX_OUTPUT_POWER                             14        /* dBm */
#define LORA_BANDWIDTH                              0         /* Hz */
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

void (*volatile eventReceptor)(LoRaHandle *const fsm);
PacketParams_t packetParams;  // TODO: this is lazy...

// IRQ callbacks

/**
 * @brief TX_DONE interrupt callback. 
 */
void radio_on_tx_done(LoRaHandle *const handle);

/**
 * @brief RX_DONE interrupt callback. The Rx buffer is readable at this
 * callback.
 */
void radio_on_rx_done(LoRaHandle *const handle);

/**
 * @brief Tx timeout callback. Should be used to report errors sending
 * data to the recipient.
 */
void radio_on_tx_timeout(LoRaHandle *const handle);

/**
 * @brief Rx timeout callback. Should be used to report when data cannot
 * be received during a data transfer. Consider sending a keep-alive ping
 * with the current data index to attempt to revive the data stream.
 */
void radio_on_rx_timeout(LoRaHandle *const handle);

/**
 * @brief Handle general errors.
 */
void radio_on_crc_error(LoRaHandle *const handle);

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void radioInit(void);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// TODO: 
// 1. Real time clock to delay packet transfers
// 2. Configure host and client by mac address
// 3. Figure out how to send/receive/manage the data
//    Note: Data will be received on the client chip via SPIq from an external board

// UART I/O minilib

/**
 * @brief Sends `printf`-formatted string to USART2 COM port and returns the
 * status code of the transmission.
 */
HAL_StatusTypeDef uprintf(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  int needed = vsnprintf(NULL, 0, fmt, args);
  
  va_start(args, fmt);
  char* uartbuf = calloc(1, needed + 1); // null byte not needed, but included for safety anyway.
  vsnprintf(uartbuf, needed, fmt, args);
  va_end(args);

  //free(uartbuf);
  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)uartbuf, needed, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, "\r\n", strlen("\r\n"), HAL_MAX_DELAY);
  
  //HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, "test\r\n", strlen("test\r\n"), HAL_MAX_DELAY);
  free(uartbuf);
  return status;
}

// # LoRa Data Transfer Protocol Structure
// TODO: Research checksum generation/evaluation

// ## Packet Structure
// - ID: 1 byte
// - Length: 1 byte
// - Body: u8[Length]

// ## Packet Body Types

// ### Handshake
// Establishes a connection between the client (sender) and host (receiver)

// Structure:
// - 

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  LoRaHandle lora_handle;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /*** GPIO Configuration (for debugging) ***/
  /* DEBUG_SUBGHZSPI_NSSOUT = PA4
   * DEBUG_SUBGHZSPI_SCKOUT = PA5
   * DEBUG_SUBGHZSPI_MISOOUT = PA6
   * DEBUG_SUBGHZSPI_MOSIOUT = PA7
   * DEBUG_RF_HSE32RDY = PA10
   * DEBUG_RF_NRESET = PA11
   * DEBUG_RF_SMPSRDY = PB2
   * DEBUG_RF_DTB1 = PB3 <---- Conflicts with RF_IRQ0
   * DEBUG_RF_LDORDY = PB4
   * RF_BUSY = PA12
   * RF_IRQ0 = PB3
   * RF_IRQ1 = PB5
   * RF_IRQ2 = PB8
   */

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO Clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // DEBUG_SUBGHZSPI_{NSSOUT, SCKOUT, MSIOOUT, MOSIOUT} pins
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_SUBGHZSPI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // DEBUG_RF_{HSE32RDY, NRESET} pins
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_RF;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // DEBUG_RF_{SMPSRDY, LDORDY} pins
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // RF_BUSY pin
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Alternate = GPIO_AF6_RF_BUSY;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // RF_{IRQ0, IRQ1, IRQ2} pins
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SUBGHZ_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  radioInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // get random number
  uint32_t rnd = 0;
  SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
  rnd = SUBGRF_GetRandom();

  lora_handle.state = STATE_NULL;
  lora_handle.subState = SSTATE_NULL;
  lora_handle.rxTimeout = 3000; // 3000 ms
  lora_handle.rxMargin = 200;   // 200 ms
  lora_handle.randomDelay = rnd >> 22; // [0, 1023] ms

  HAL_Delay(lora_handle.randomDelay);
  SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  SUBGRF_SetRx(lora_handle.rxTimeout << 6);
  lora_handle.state = STATE_SENDER;
  lora_handle.subState = SSTATE_RX;

  uprintf("Starting...\r\n");
  //uprintf("\r\ntest\r\n");


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    eventReceptor = NULL;
    while (eventReceptor == NULL);
    eventReceptor(&lora_handle);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Initialize the Sub-GHz radio and dependent hardware.
  * @retval None
  */
void radioInit(void)
{
  // Initialize the hardware (SPI bus, TCXO control, RF switch)
  SUBGRF_Init(RadioOnDioIrq);

  // Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
  // "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
  SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
  SUBGRF_SetRegulatorMode();

  // Use the whole 256-byte buffer for both TX and RX
  SUBGRF_SetBufferBaseAddress(0x00, 0x00);

  SUBGRF_SetRfFrequency(RF_FREQUENCY);
  SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
  SUBGRF_SetStopRxTimerOnPreambleDetect(false);

  SUBGRF_SetPacketType(PACKET_TYPE_LORA);

  SUBGRF_WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
  SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

  ModulationParams_t modulationParams;
  modulationParams.PacketType = PACKET_TYPE_LORA;
  modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
  modulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)LORA_CODINGRATE;
  modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
  modulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
  SUBGRF_SetModulationParams(&modulationParams);

  packetParams.PacketType = PACKET_TYPE_LORA;
  packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
  packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
  packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
  SUBGRF_SetPacketParams(&packetParams);

  //SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

  // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
  // RegIqPolaritySetup @address 0x0736
  SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );
}


// IRQ callbacks
void radio_on_tx_done(LoRaHandle *const handle) {
  uprintf("Transfer complete.\n");
}

void radio_on_rx_done(LoRaHandle *const handle) {
  uprintf("Recv complete.\n");
}

void radio_on_tx_timeout(LoRaHandle *const handle) {
  uprintf("Error: Transfer timeout.\n");
}

void radio_on_rx_timeout(LoRaHandle *const handle) {
  uprintf("Error: Recv timeout.\n");
}

void radio_on_crc_error(LoRaHandle *const handle) {
  uprintf("Error: General Recv error.\n");
}


/**
  * @brief  Receive data trough SUBGHZSPI peripheral
  * @param  radioIrq  interrupt pending status information
  * @retval None
  */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
  uprintf("incoming radio interrupt...\n");
  switch (radioIrq)
  {
    case IRQ_TX_DONE:
      eventReceptor = radio_on_tx_done;
      break;
    case IRQ_RX_DONE:
      eventReceptor = radio_on_rx_done;
      break;
    case IRQ_RX_TX_TIMEOUT:
      if (SUBGRF_GetOperatingMode() == MODE_TX) {
        eventReceptor = radio_on_tx_timeout;
      } else if (SUBGRF_GetOperatingMode() == MODE_RX) {
        eventReceptor = radio_on_rx_timeout;
      }
      break;
    case IRQ_CRC_ERROR:
      eventReceptor = radio_on_crc_error;
      break;
    default:
      __builtin_unreachable();
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/