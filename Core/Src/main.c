/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus_crc.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t txdata[8];
uint8_t rxdata[32];
uint16_t regValues[10];
uint8_t parity_err=0;
int PEcount,NEcount=0;
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/// 1. Global variables to track parity errors and counts
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void UPDATEUSART3(void);
void recvString()
{
	HAL_GPIO_WritePin(TX_EN_GPIO_Port,TX_EN_Pin , GPIO_PIN_RESET); // only accept the string
}

void send_rs485() //uart 1
{
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin , GPIO_PIN_SET); //send the accepted string
}

void sendQuery (uint8_t *data_,uint16_t len) //com5 only  to write
{
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);//sending mode
	HAL_UART_Transmit(&huart3, data_, len, 1000);
	HAL_GPIO_WritePin(TX_EN_GPIO_Port,TX_EN_Pin , GPIO_PIN_RESET); //receiving mode
}

void sendData (uint8_t *data_,uint16_t len) //com5 only  to write
{
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);//sending mode
	HAL_UART_Transmit(&huart3, data_, len, 1000);
	HAL_GPIO_WritePin(TX_EN_GPIO_Port,TX_EN_Pin , GPIO_PIN_RESET); //receiving mode
}

void modbus_write_multiple_coils(uint8_t *coilStates, uint16_t coilCount, uint8_t slaveAddr, uint16_t startAddr) {
    uint8_t TxData[260];
    uint8_t idx = 0;
    uint8_t byteCount = (coilCount + 7) / 8;

    TxData[idx++] = slaveAddr;
    TxData[idx++] = 0x0F;  // Function code
    TxData[idx++] = (startAddr >> 8);
    TxData[idx++] = (startAddr & 0xFF);
    TxData[idx++] = (coilCount >> 8);
    TxData[idx++] = (coilCount & 0xFF);
    TxData[idx++] = byteCount;

    memcpy(&TxData[idx], coilStates, byteCount);
    idx += byteCount;

    uint16_t crc = crc16(TxData, idx);
    TxData[idx++] = crc & 0xFF;
    TxData[idx++] = (crc >> 8);

    sendData(TxData, idx);

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void functionREADcoils(uint8_t slaveID, uint16_t startREGadd, uint16_t datatoREAD)
  {
txdata[0]=slaveID;
txdata[1]=0x01;
txdata[2]=(startREGadd >> 8);
txdata[3]=(startREGadd & 0xFF);
txdata[4]=(datatoREAD >> 8);
txdata[5]=(datatoREAD & 0xFF);
uint16_t crc=crc16(txdata,6);
txdata[6]=crc & 0xFF;
txdata[7]=(crc>>8)&0xFF;
sendQuery(txdata,sizeof(txdata));
  }
void functionREADinputstatus_register(uint8_t slaveID, uint16_t startREGadd, uint16_t datatoREAD)
  {
txdata[0]=slaveID;
txdata[1]=0x02;
txdata[2]=(startREGadd >> 8);
txdata[3]=(startREGadd & 0xFF);
txdata[4]=(datatoREAD >> 8);
txdata[5]=(datatoREAD & 0xFF);
uint16_t crc=crc16(txdata,6);
txdata[6]=crc & 0xFF;
txdata[7]=(crc>>8)&0xFF;
sendQuery(txdata,sizeof(txdata));
  }
void functionREADholding_register(uint8_t slaveID, uint16_t startREGadd, uint16_t datatoREAD)
  {
txdata[0]=slaveID;
txdata[1]=0x03;
txdata[2]=(startREGadd >> 8);
txdata[3]=(startREGadd & 0xFF);
txdata[4]=(datatoREAD >> 8);
txdata[5]=(datatoREAD & 0xFF);
uint16_t crc=crc16(txdata,6);
txdata[6]=crc & 0xFF;
txdata[7]=(crc>>8)&0xFF;
sendQuery(txdata,sizeof(txdata));
  }
void functionREADinput_register(uint8_t slaveID, uint16_t startREGadd, uint16_t datatoREAD)
  {
txdata[0]=slaveID;
txdata[1]=0x04;
txdata[2]=(startREGadd >> 8);
txdata[3]=(startREGadd & 0xFF);
txdata[4]=(datatoREAD >> 8);
txdata[5]=(datatoREAD & 0xFF);
uint16_t crc=crc16(txdata,6);
txdata[6]=crc & 0xFF;
txdata[7]=(crc>>8)&0xFF;
sendQuery(txdata,sizeof(txdata));
  }

void functionWRITEholding_register(uint8_t slaveID, uint16_t startREGadd, uint16_t datatoWRITE, uint16_t* values)
{
    // The request frame length is: ID(1) + FC(1) + Addr(2) + Quantity(2) + ByteCount(1) + Data(N*2) + CRC(2)
    uint16_t request_len = 9 + (datatoWRITE * 2);
    uint8_t writeTxData[request_len];
    uint8_t current_len = 0;

    // 1. Header (7 bytes: ID, FC, Addr, Quantity, Byte Count)
    writeTxData[current_len++] = slaveID; // Slave Address
    writeTxData[current_len++] = 0x10;    // Function Code: 16 (Write Multiple Registers)
    writeTxData[current_len++] = (startREGadd >> 8);
    writeTxData[current_len++] = (startREGadd & 0xFF);
    writeTxData[current_len++] = (datatoWRITE >> 8); // Quantity High
    writeTxData[current_len++] = (datatoWRITE & 0xFF); // Quantity Low

    uint8_t byte_count = datatoWRITE * 2;
    writeTxData[current_len++] = byte_count; // Byte Count

    // 2. Data Payload (N * 2 bytes)
    for (int i = 0; i < datatoWRITE; i++)
    {
        // Place the 16-bit value from 'values' into the buffer (Big-Endian)
        writeTxData[current_len++] = (values[i] >> 8);  // High Byte
        writeTxData[current_len++] = (values[i] & 0xFF); // Low Byte
    }

    // 3. CRC Checksum (2 bytes)
    uint16_t crc = crc16(writeTxData, current_len);
    writeTxData[current_len++] = crc & 0xFF;        // CRC Low
    writeTxData[current_len++] = (crc >> 8) & 0xFF; // CRC High

    printf("Modbus Write Request (FC 16, %d bytes): ", current_len);

        // Loop through the buffer up to the final calculated length (including CRC)
        for (int i = 0; i < current_len; i++)
        {
            // %02X ensures two hexadecimal characters are printed (e.g., 5B instead of B)
            printf("%02X ", writeTxData[i]);
        }
        printf("\r\n");
    // Transmit the command using the generic send function
    sendData(writeTxData, current_len);
}
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  UPDATEUSART3();
  printf("TESTING>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxdata, 32);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  functionREADholding_register(0x01,0x00,5);
	  HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TX_EN_Pin */
  GPIO_InitStruct.Pin = TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TX_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//	HAL_UARTEx_ReceiveToIdle_IT(&huart1, rxdata, 32);
	if (huart->Instance == huart3.Instance)
	    {
	        // 'Size' is the exact number of data bytes received.
	        printf("UART Received (%d bytes): ", Size);
	        // Use a printing function that iterates exactly up to 'Size'
	        for (int i = 0; i < Size; i++)
	            printf("%02X ", rxdata[i]);
	        printf("\r\n");
	        HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxdata, sizeof(rxdata));

	        // --- MODBUS RESPONSE PROCESSING AND WRITE LOGIC ---

	                // Check if the response is from Slave 01 and is a Read Holding Register response (FC 03)
			if (rxdata[0] == 0x01 && rxdata[1] == 0x03)
			{
				uint8_t byte_count = rxdata[2];
				uint8_t num_regs = byte_count / 2; // Quantity of 16-bit registers

				// Limit extraction to the buffer size
				if (num_regs > 10) num_regs = 10;

				// Extract the data starting at rxdata[3] (first data byte)
				for (int i = 0; i < num_regs; i++) {
					// (High Byte << 8) | Low Byte (Modbus data is Big-Endian)
					regValues[i] = (rxdata[3 + 2*i] << 8) | rxdata[4 + 2*i];
				}

				printf("Extracted %d registers. Writing to new destination...\r\n", num_regs);

				// *** CALL THE WRITE FUNCTION HERE ***
				// Example: Write the received data to Slave ID 2, starting at address 200
				functionWRITEholding_register(
					0x02,             // Target Slave ID (e.g., 2)
					0x0001,           // Target Start Address (e.g., 200 decimal = 0x00C8)
					num_regs,         // Quantity of registers to write
					regValues         // Pointer to the data array
				);
			}
			else if(rxdata[0] == 0x01 && rxdata[1] == 0x01)
			{
				uint8_t byte_count = rxdata[2];
				uint8_t *coil_data_start = &rxdata[3];
				modbus_write_multiple_coils(coil_data_start,5,0x02,0);
			}

	    }
}
//void print_hex(uint8_t *buf, uint16_t len)
//{
//    // Ensure this function uses huart2/printf to output the hex string
//    for (int i = 0; i < len; i++)
//        printf("%02X ", buf[i]);
//    printf("\r\n");
//}
void UPDATEUSART3(void)
{
	   // De-initialize first
	    if (HAL_UART_DeInit(&huart3) != HAL_OK) {
	        Error_Handler();
	    }
    // 1. Convert Baud string to a number
    // strtoul converts "115200" to 115200
    huart3.Init.BaudRate = 9600;

//    // 2. Map Parity string to HAL Constants
//    if (strcmp(parity, "EVEN") == 0)
//    {
//        huart3.Init.Parity = UART_PARITY_EVEN;
//        huart3.Init.WordLength = UART_WORDLENGTH_9B;
//    }
//    else if (strcmp(parity, "ODD") == 0)
//    {
//        huart3.Init.Parity = UART_PARITY_ODD;
//        huart3.Init.WordLength = UART_WORDLENGTH_9B;
//    }
//    else
//    {
//        huart3.Init.Parity = UART_PARITY_NONE;
//        huart3.Init.WordLength = UART_WORDLENGTH_8B;
//    }
//
//    // 3. Map Stopbit string to HAL Constants
//    if (strcmp(stopbit, "2") == 0)
//    {
//        huart3.Init.StopBits = UART_STOPBITS_2;
//    }
//    else
//    {
//        huart3.Init.StopBits = UART_STOPBITS_1;
//    }
//    if (strcmp(parity, "EVEN") == 0)
//    {
//    	huart3.Init.Parity = UART_PARITY_EVEN;
//    	huart3.Init.WordLength = UART_WORDLENGTH_9B;
//    }
//    else if(strcmp(parity, "ODD") == 0)
//    {
//    	huart3.Init.Parity = UART_PARITY_ODD;
//    	huart3.Init.WordLength = UART_WORDLENGTH_9B;
//    }
//    else
//        {
//            huart3.Init.Parity = UART_PARITY_NONE;
//            huart3.Init.WordLength = UART_WORDLENGTH_8B;
//        }
    huart3.Init.Parity = UART_PARITY_ODD;
	huart3.Init.WordLength = UART_WORDLENGTH_9B;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    	   {
    		  Error_Handler();
    	   }
    if(HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxdata, sizeof(rxdata))!=HAL_OK)
    {
    	Error_Handler();
    }
}
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->ErrorCode & HAL_UART_ERROR_PE)
//    {
//        parity_err = 1;
//    }
//    else if (huart->ErrorCode & HAL_UART_ERROR_NONE)
//        {
//            parity_err = 2;
//        }
//}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 1. Check for Parity Error specifically
    if (huart->ErrorCode & HAL_UART_ERROR_PE)
    {
        parity_err = 1; // Mark that we had a parity issue
        PEcount++;
        __HAL_UART_CLEAR_PEFLAG(huart);
//        NEcount=0;
    }
    else if(huart->ErrorCode & (HAL_UART_ERROR_NE | HAL_UART_ERROR_NONE|HAL_UART_ERROR_FE|HAL_UART_ERROR_ORE))
    {
    	__HAL_UART_CLEAR_PEFLAG(huart);
    	parity_err=0;
    }

    // 2. Handle other common errors (Overrun, Frame, etc.)
//    else if (huart->ErrorCode & HAL_UART_ERROR_NONE)
//    {
//        // These are timing or hardware connection issues
//        parity_err = 0;
//    	PEcount=0;
//    	NEcount++;
//    }

}



PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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

