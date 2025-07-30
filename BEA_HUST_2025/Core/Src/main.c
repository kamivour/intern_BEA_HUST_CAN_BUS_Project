/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "iso_tp.h"
#include "dcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t uart3_receive;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef CAN1_pHeader;
CAN_RxHeaderTypeDef CAN1_pHeaderRx;
CAN_FilterTypeDef CAN1_sFilterConfig;
CAN_TxHeaderTypeDef CAN2_pHeader;
CAN_RxHeaderTypeDef CAN2_pHeaderRx;
CAN_FilterTypeDef CAN2_sFilterConfig;
uint32_t CAN1_pTxMailbox;
uint32_t CAN2_pTxMailbox;
uint16_t NumBytesReq = 0;
uint8_t REQ_BUFFER[4096];
uint8_t REQ_1BYTE_DATA;
uint8_t SEED[6] = { 0x01, 0x08, 0x82, 0x21, 0xAB, 0xCD };
uint8_t KEY[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t CAN1_DATA_TX[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t CAN1_DATA_RX[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t CAN2_DATA_TX[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t CAN2_DATA_RX[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint16_t Num_Consecutive_Tester;
uint8_t Flg_Consecutive = 0;
volatile uint8_t check;
unsigned int TimeStamp;
volatile uint8_t SecurityUnlocked = 0;
uint8_t SeedProvided = 0;
uint32_t newStdID;
uint16_t AVAILABLE_SERVICE = 0x0123;  // Data Identifier
// ISO-TP variables
uint16_t iso_tp_rx_len;             // Expected total length for RX (used by DCM modules)

// maximum characters send out via UART is 40
char bufsend[40] = "XXX: D1 D2 D3 D4 D5 D6 D7 D8  ";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void MX_CAN1_Setup();
void MX_CAN2_Setup();
void USART3_SendString(uint8_t *ch);
void send_uds_request_via_can1(uint8_t *uds_data, uint16_t length);
void PrintCANLog(uint16_t CANID, uint8_t *CAN_Frame);
void delay(uint16_t delay);
void read_from_buffer(uint8_t *req_buffer, uint16_t len, uint8_t *data_tx);
void CAN1CommSetup();
void CAN2CommSetup();
void CAN1_Send();
void CAN2_Send();
void calculate_key(uint8_t *input, uint8_t *output);
uint8_t compare_key(uint8_t *array1, uint8_t *array2, uint8_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	//uint16_t i, j = 0;
	//uint16_t Consecutive_Cntr = 0;
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
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	CAN1CommSetup();
	CAN2CommSetup();
	MX_CAN1_Setup();
	MX_CAN2_Setup();
	iso_tp_init();  // Initialize ISO-TP
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	// Example Function to print can message via uart
	PrintCANLog(CAN1_pHeader.StdId, &CAN1_DATA_TX[0]);
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// Update ISO-TP timer
		iso_tp_timer_update();
		
		// Check if CAN1 received a response (non-Flow Control) for display
		if (check == 1) {
			USART3_SendString((uint8_t*) "Response: ");
			PrintCANLog(CAN1_pHeaderRx.StdId, CAN1_DATA_RX);
			check = 0;
		}
		
		if (NumBytesReq != 0) {
			delay(100);
			
			// Convert raw UDS data to ISO-TP frames
			// User sends: "27 01" → Program creates: "02 27 01"
			send_uds_request_via_can1(REQ_BUFFER, NumBytesReq);
			
			// Clear buffer after processing
			memset(REQ_BUFFER, 0, sizeof(REQ_BUFFER));
			NumBytesReq = 0;
			
			delay(100);
		}
		if (!BtnU) /*IG OFF->ON stimulation*/
		{
			delay(20);
			USART3_SendString((uint8_t*) "IG OFF ");
			while (!BtnU)
				;
			CAN1_pHeader.StdId = newStdID;
			CAN2CommSetup();
			MX_CAN1_Setup();
			MX_CAN2_Setup();
			USART3_SendString((uint8_t*) "-> IG ON \n");
			delay(20);
		}
	}

	memset(&REQ_BUFFER, 0x00, 4096);
	NumBytesReq = 0;

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 80;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 21;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void) {

	/* USER CODE BEGIN CAN2_Init 0 */

	/* USER CODE END CAN2_Init 0 */

	/* USER CODE BEGIN CAN2_Init 1 */

	/* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 21;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_12TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */

	/* USER CODE END CAN2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 7999;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 49999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC4 PC5 PC6
	 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
			| GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void read_from_buffer(uint8_t *req_buffer, uint16_t len, uint8_t *data_tx) {
	// Copy the complete ISO-TP frame including PCI
	uint8_t i;
	for (i = 0; i < len && i < 8; i++) {
		data_tx[i] = req_buffer[i];
	}
	// Pad remaining bytes with 0x55
	while (i < 8) {
		data_tx[i] = 0x55;
		i++;
	}
}
void MX_CAN1_Setup() {
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void MX_CAN2_Setup() {
	HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void USART3_SendString(uint8_t *ch) {
	while (*ch != 0) {
		if (*ch == '\n') {
			// Send carriage return before newline for proper line ending
			unsigned char cr = '\r';
			HAL_UART_Transmit(&huart3, &cr, 1, HAL_MAX_DELAY);
		}
		HAL_UART_Transmit(&huart3, ch, 1, HAL_MAX_DELAY);
		ch++;
	}
}
void PrintCANLog(uint16_t CANID, uint8_t *CAN_Frame) {
	// Print timestamp
	sprintf(bufsend, "%d ", TimeStamp);
	USART3_SendString((uint8_t*) bufsend);
	
	// Format and print the complete CAN message: "XXX: D1 D2 D3 D4 D5 D6 D7 D8 "
	sprintf(bufsend, "0x%03X: %02X %02X %02X %02X %02X %02X %02X %02X \n", 
			CANID, 
			CAN_Frame[0], CAN_Frame[1], CAN_Frame[2], CAN_Frame[3],
			CAN_Frame[4], CAN_Frame[5], CAN_Frame[6], CAN_Frame[7]);
	USART3_SendString((unsigned char*) bufsend);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	REQ_BUFFER[NumBytesReq] = REQ_1BYTE_DATA;
	NumBytesReq++;
	//REQ_BUFFER[7] = NumBytesReq;
}
void delay(uint16_t delay) {
	HAL_Delay(delay);
}

// Correct calculated key for SEED {0x01, 0x08, 0x82, 0x21, 0xAB, 0xCD} is: {0x09, 0x8A, 0xA3, 0x22, 0xA0, 0x0D}
// UART command: "27 02 09 8A A3 22 A0 0D"
void calculate_key(uint8_t *input, uint8_t *output) {
	output[0] = input[0] ^ input[1];
	output[1] = input[1] + input[2];
	output[2] = input[2] ^ input[3];
	output[3] = input[3] + input[0];
	output[4] = input[4] & 0xF0;
	output[5] = input[5] & 0x0F;
}

// Function to send UDS request via CAN1 with proper ISO-TP framing
void send_uds_request_via_can1(uint8_t *uds_data, uint16_t length) {
	if (length == 0 || length > 4095) {
		USART3_SendString((uint8_t*) "Invalid UDS data length\n");
		return;
	}
	
	// Print debug info
	char debug_msg[100];
	/*
	sprintf(debug_msg, "UDS Request (%d bytes): ", length);
	USART3_SendString((uint8_t*) debug_msg);
	
	for (int i = 0; i < length && i < 10; i++) {
		sprintf(debug_msg, "%02X ", uds_data[i]);
		USART3_SendString((uint8_t*) debug_msg);
	}
	USART3_SendString((uint8_t*) "\n");
	*/
	
	if (length <= 7) {
		// Single Frame: First byte = 0x0L (L = length)
		CAN1_DATA_TX[0] = 0x00 | length; // Single Frame PCI + length
		memcpy(&CAN1_DATA_TX[1], uds_data, length);
		
		// Pad remaining bytes
		for (int i = length + 1; i < 8; i++) {
			CAN1_DATA_TX[i] = 0x55;
		}
		
		//sprintf(debug_msg, "Sending Single Frame: ");
		//USART3_SendString((uint8_t*) debug_msg);
		//PrintCANLog(CAN1_pHeader.StdId, CAN1_DATA_TX);

		USART3_SendString((uint8_t*) "Request: ");
		CAN1_Send();
	} else {
		// Multi-Frame: First Frame + Consecutive Frames
		// First Frame: 1L LL DD DD DD DD DD DD (L LL = 12-bit length)
		CAN1_DATA_TX[0] = 0x10 | ((length >> 8) & 0x0F); // FF PCI + high nibble of length
		CAN1_DATA_TX[1] = length & 0xFF; // Low byte of length
		memcpy(&CAN1_DATA_TX[2], uds_data, 6); // First 6 bytes of data
		
		sprintf(debug_msg, "Sending First Frame: ");
		USART3_SendString((uint8_t*) debug_msg);
		PrintCANLog(CAN1_pHeader.StdId, CAN1_DATA_TX);
		
		CAN1_Send();
		
		// Wait for Flow Control (in a real implementation, this should be event-driven)
		delay(100);
		
		// Send Consecutive Frames
		uint16_t sent_bytes = 6;
		uint8_t sequence_number = 1;
		
		while (sent_bytes < length) {
			uint8_t remaining = length - sent_bytes;
			uint8_t to_send = (remaining > 7) ? 7 : remaining;
			
			CAN1_DATA_TX[0] = 0x20 | (sequence_number & 0x0F); // CF PCI + sequence number
			memcpy(&CAN1_DATA_TX[1], &uds_data[sent_bytes], to_send);
			
			// Pad remaining bytes
			for (int i = to_send + 1; i < 8; i++) {
				CAN1_DATA_TX[i] = 0x55;
			}
			
			sprintf(debug_msg, "Sending Consecutive Frame %d: ", sequence_number);
			USART3_SendString((uint8_t*) debug_msg);
			PrintCANLog(CAN1_pHeader.StdId, CAN1_DATA_TX);
			
			CAN1_Send();
			
			sent_bytes += to_send;
			sequence_number++;
			if (sequence_number > 15) sequence_number = 0; // Wrap around
			
			delay(50); // Small delay between frames
		}
	}
}
uint8_t compare_key(uint8_t *array1, uint8_t *array2, uint8_t length) {
	for (uint8_t i = 0; i < length; i++) {
		if (array1[i] != array2[i]) {
			return 0;
		}
	}
	return 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_StatusTypeDef ret;
	if (hcan == &hcan1) {
		ret = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_pHeaderRx,
				CAN1_DATA_RX);
		if (ret != HAL_OK) {
			Error_Handler();
		}
		
		// Process CAN1 frames immediately in interrupt
		uint8_t pci = CAN1_DATA_RX[0] & 0xF0;
		if (pci == ISO_TP_PCI_FC) {
			// Process Flow Control from ECU
			iso_tp_process_rx(CAN1_DATA_RX);
		} else {
			// Set flag for main loop to handle response display
			check = 1;
		}
		return;
	}
	if (hcan == &hcan2) {
		ret = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_pHeaderRx,
				CAN2_DATA_RX);
		if (ret != HAL_OK) {
			Error_Handler();
		}
		
		// Process CAN2 frames immediately in interrupt
		iso_tp_process_rx(CAN2_DATA_RX);
		return;
	}
}

void CAN1_Send() {
	PrintCANLog(CAN1_pHeader.StdId, CAN1_DATA_TX);
	if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX,
			&CAN1_pTxMailbox) != HAL_OK) {
		Error_Handler();
	}
}

void CAN1CommSetup() {
	CAN1_pHeader.IDE = CAN_ID_STD;
	CAN1_pHeader.StdId = 0x712;
	CAN1_pHeader.RTR = CAN_RTR_DATA;
	CAN1_pHeader.DLC = 8;
	CAN1_sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	CAN1_sFilterConfig.FilterBank = 14;
	CAN1_sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN1_sFilterConfig.FilterIdHigh = 0x7A2 << 5;
	CAN1_sFilterConfig.FilterIdLow = 0;
	CAN1_sFilterConfig.FilterMaskIdHigh = 0x7A2 << 5;
	CAN1_sFilterConfig.FilterMaskIdLow = 0;
	CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN1_sFilterConfig.SlaveStartFilterBank = 16;
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig);
}

void CAN2_Send() {
	PrintCANLog(CAN2_pHeader.StdId, CAN2_DATA_TX);
	if (HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX,
			&CAN2_pTxMailbox) != HAL_OK) {
		Error_Handler();
	}
}

void CAN2CommSetup() {
	CAN2_pHeader.IDE = CAN_ID_STD;
	CAN2_pHeader.StdId = 0x7A2;
	CAN2_pHeader.RTR = CAN_RTR_DATA;
	CAN2_pHeader.DLC = 8;
	CAN2_sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	CAN2_sFilterConfig.FilterBank = 18;
	CAN2_sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN2_sFilterConfig.FilterIdHigh = CAN1_pHeader.StdId << 5;
	CAN2_sFilterConfig.FilterIdLow = 0;
	CAN2_sFilterConfig.FilterMaskIdHigh = CAN1_pHeader.StdId << 5;
	CAN2_sFilterConfig.FilterMaskIdLow = 0;
	CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		if (SecurityUnlocked == 0)
			return;
		else {
			HAL_TIM_Base_Stop_IT(htim);
			SecurityUnlocked = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			USART3_SendString((uint8_t*) "Session Locked\n");
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
