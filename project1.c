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
uint8_t  REQ_BUFFER  [4096];
uint8_t  REQ_1BYTE_DATA;

uint8_t CAN1_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN1_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN2_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN2_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint16_t Num_Consecutive_Tester;
uint8_t  Flg_Consecutive = 0;

//uint8_t g_can_rx_data[8];
//uint8_t g_is_msg_received = 0;

unsigned int TimeStamp;
// maximum characters send out via UART is 30
char bufsend[30]="XXX: D1 D2 D3 D4 D5 D6 D7 D8  ";

uint8_t MessageCounter = 0;
uint8_t FlagForBtn = 0;
uint8_t flagCAN2Rx = 0;
uint8_t CAN1_DataReceived = 0;  // ✅ Flag để tracking data received
uint8_t CAN2_DataReceived = 0;  // Flag tracking message 0x012 received
uint8_t CAN2_LastCRCValid = 0;  // Flag tracking CRC valid của message 0x012
uint8_t ledState = 0;  // LED state tracking
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void MX_CAN1_Setup();
void MX_CAN2_Setup();
void USART3_SendString(uint8_t *ch);
void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame);
void delay(uint16_t delay);
//void SID_22_Practice();
//void SID_2E_Practice();
//void SID_27_Practice();
void delay(uint16_t delay);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t calc_SAE_J1850(uint8_t data[], uint8_t crc_len, uint8_t check);
void CAN_Tx(CAN_HandleTypeDef *hcan, uint8_t *data_rx, uint8_t *data_tx);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint16_t i,j = 0;
//	uint16_t Consecutive_Cntr = 0;
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
  /* USER CODE BEGIN 2 */
  USART3_SendString((uint8_t*)"Board da khoi dong\n");
  HAL_Delay(1000);

  MX_CAN1_Setup();
  MX_CAN2_Setup();

  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t cycleStart = HAL_GetTick();  // ✅ Đo thời gian chính xác
    
    // Node 2 gửi trước
    CAN_Tx(&hcan2, CAN2_DATA_RX, CAN2_DATA_TX);
    HAL_Delay(100);  // ✅ Delay ngắn để Node 1 nhận được
    
    // Node 1 gửi phản hồi
    CAN_Tx(&hcan1, CAN1_DATA_RX, CAN1_DATA_TX);
    
    MessageCounter = (MessageCounter + 1) & 0xF;  // ✅ Đảm bảo 4-bit counter
    
    // ✅ Đảm bảo chu kỳ chính xác 4000ms
    uint32_t elapsedTime = HAL_GetTick() - cycleStart;
    if (elapsedTime < 4000) {
        uint32_t remainingTime = 4000 - elapsedTime;
        if (remainingTime > 0) {  // ✅ Thêm check an toàn
            HAL_Delay(remainingTime);
        }
    }
}

  memset(&REQ_BUFFER,0x00,4096); // clear request buffer
  NumBytesReq = 0; // reset number of bytes in request buffer

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	CAN1_pHeader.StdId = 0x12;
	CAN1_pHeader.DLC = 8;
	CAN1_pHeader.IDE = CAN_ID_STD;
	CAN1_pHeader.RTR = CAN_RTR_DATA;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;              // ✅ Sửa lại: 42MHz/(16×500kHz) = 5.25 ≈ 5
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ; // ✅ Sửa lại: 2 Tq theo yêu cầu
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;    // ✅ Sửa lại: 11 Tq cho 75% sampling
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;     // ✅ Giữ nguyên: 4 Tq
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN1_sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  CAN1_sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN1_sFilterConfig.SlaveStartFilterBank = 13;
  CAN1_sFilterConfig.FilterBank = 8;
  CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  CAN1_sFilterConfig.FilterIdHigh = 0x0A2 << 5;
  CAN1_sFilterConfig.FilterIdLow = 0;
  CAN1_sFilterConfig.FilterMaskIdHigh = 0x0A2 << 5;
  CAN1_sFilterConfig.FilterMaskIdLow = 0;
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
	CAN2_pHeader.StdId = 0xA2;
	CAN2_pHeader.DLC = 8;
	CAN2_pHeader.IDE = CAN_ID_STD;
	CAN2_pHeader.RTR = CAN_RTR_DATA;
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;              // ✅ Sửa lại: giống CAN1
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ; // ✅ Sửa lại: 2 Tq theo yêu cầu
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;    // ✅ Sửa lại: 11 Tq cho 75% sampling
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;     // ✅ Giữ nguyên: 4 Tq
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN2_sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  CAN2_sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN2_sFilterConfig.SlaveStartFilterBank = 13;
  CAN2_sFilterConfig.FilterBank = 19;
  CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  CAN2_sFilterConfig.FilterIdHigh = 0x012 << 5;
  CAN2_sFilterConfig.FilterIdLow = 0;
  CAN2_sFilterConfig.FilterMaskIdHigh = 0x012 << 5;
  CAN2_sFilterConfig.FilterMaskIdLow = 0;
  /* USER CODE END CAN2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC4 PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // ✅ SỬA: Config PA0 cho button (USER button trên board)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Falling edge trigger
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // Pull-up enabled
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // ✅ SỬA: Config PB1 cho LED (thay vì PB0 để tránh conflict)
  GPIO_InitStruct.Pin = GPIO_PIN_1;             // ✅ Dùng PB1 thay vì PB0
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // ✅ PB1

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */


void MX_CAN1_Setup()
{
  HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void MX_CAN2_Setup()
{
	HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void USART3_SendString(uint8_t *ch)
{
   while(*ch!=0)
   {
      HAL_UART_Transmit(&huart3, ch, 1,HAL_MAX_DELAY);
      ch++;
   }
}
void PrintCANLog(uint16_t CAN_ID, uint8_t * CAN_Frame)
{
	uint16_t loopIndx = 0;
		char bufID[3] = "   "; // `3 characters for ID, 2 for data, 1 for ':'
		char bufDat[2] = "  "; // `2 characters for data
		char bufTime [8]="        "; // `8 characters for timestamp

		sprintf(bufTime,"%d",TimeStamp); 
		USART3_SendString((uint8_t*)bufTime);
		USART3_SendString((uint8_t*)" ");

		sprintf(bufID,"%03X",CAN_ID);
		for(loopIndx = 0; loopIndx < 3; loopIndx ++)
		{
			bufsend[loopIndx]  = bufID[loopIndx];
		}
		bufsend[3] = ':';
		bufsend[4] = ' ';

		for(loopIndx = 0; loopIndx < 8; loopIndx ++ )
		{
			sprintf(bufDat,"%02X",CAN_Frame[loopIndx]);
			bufsend[loopIndx*3 + 5] = bufDat[0];
			bufsend[loopIndx*3 + 6] = bufDat[1];
			bufsend[loopIndx*3 + 7] = ' ';
		}
		bufsend[29] = '\n';
		USART3_SendString((unsigned char*)bufsend);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	REQ_BUFFER[NumBytesReq] = REQ_1BYTE_DATA; // store the received byte
	NumBytesReq++; // increment the number of bytes in request buffer
	//REQ_BUFFER[7] = NumBytesReq;
}
void delay(uint16_t delay)
{
	HAL_Delay(delay);
}
uint8_t calc_SAE_J1850(uint8_t data[], uint8_t crc_len, uint8_t check) // check = 0 for calculating CRC, check = 1 for checking CRC
{
    uint8_t idx, crc, temp1, temp2, idy; 
    crc = check;
    idx = 0; //index for main loop
    idy = 0; // index for inner loop
    temp1 = 0; //byte to be processed
    temp2 = 0; // backup of crc value
    for(idx=0;idx < crc_len+1;idx++)
    {
        if(idx == 0)
        {
            temp1 = 0;
        }
        else
        {
            temp1 = data[crc_len-idx];
        }
        crc = (crc^temp1); // XOR the current byte with the crc value
        for(idy=(uint8_t)8; idy>0; idy--)
        {
            // Save the value before the top bit is shifted out.
            temp2 = crc;
            crc <<= 1;
            if (0 != (temp2 & (uint8_t)128))
                crc ^= 0x1D;
        }
    }
    return crc;
}

void CAN_Tx(CAN_HandleTypeDef *hcan, uint8_t *data_rx, uint8_t *data_tx){

    // Clear all data
    memset(data_tx, 0x00, 8);
    
    if(hcan == &hcan1) {
        // Node 1: Xử lý message 0x012 dựa trên data từ Node 2
        
        // ✅ Logic đơn giản: copy data từ CAN1_DATA_RX (nhận từ Node 2)
        data_tx[0] = CAN1_DATA_RX[0];  // Copy byte 0 từ message 0x0A2
        data_tx[1] = CAN1_DATA_RX[1];  // Copy byte 1 từ message 0x0A2
        data_tx[2] = data_tx[0] + data_tx[1];  // Tính tổng bytes 0+1
        data_tx[3] = 0x00;
        data_tx[4] = 0x00; 
        data_tx[5] = 0x00;
        data_tx[6] = MessageCounter;
        
        // ✅ Xử lý error injection đơn giản
        if(FlagForBtn == 0) {
            data_tx[7] = calc_SAE_J1850(data_tx, 7, 0);  // CRC đúng
        } else {
            data_tx[7] = calc_SAE_J1850(data_tx, 7, 0) ^ 0xFF;  // CRC sai
            USART3_SendString((uint8_t *)"INJECTING CRC ERROR FOR 0x012\n");
        }
    }
    else if(hcan == &hcan2) {
        // ✅ Node 2: Xử lý lỗi khi không nhận được message 0x012 hoặc CRC sai
        
        if(CAN2_DataReceived && CAN2_LastCRCValid) {
            // ✅ Nhận được message 0x012 và CRC đúng → gửi data bình thường
            data_tx[0] = 0x01;
            data_tx[1] = 0x02;
            data_tx[2] = 0x00;
            data_tx[3] = 0x00;
            data_tx[4] = 0x00;
            data_tx[5] = 0x00;
            USART3_SendString((uint8_t *)"Node 2: Normal operation\n");
        } else {
            // ✅ KHÔNG nhận được message 0x012 hoặc CRC sai → gửi 0x00
            data_tx[0] = 0x00;  // Error state
            data_tx[1] = 0x00;  // Error state
            data_tx[2] = 0x00;  // Error state
            data_tx[3] = 0x00;  // Error state
            data_tx[4] = 0x00;  // Error state
            data_tx[5] = 0x00;  // Error state
            
            if(!CAN2_DataReceived) {
                USART3_SendString((uint8_t *)"Node 2: No message 0x012 received - sending 0x00\n");
            } else if(!CAN2_LastCRCValid) {
                USART3_SendString((uint8_t *)"Node 2: CRC error in 0x012 - sending 0x00\n");
            }
        }
        
        data_tx[6] = MessageCounter;
        data_tx[7] = calc_SAE_J1850(data_tx, 7, 0);  // CRC cho message 0x0A2
        
        // ✅ Reset flags sau khi xử lý
        CAN2_DataReceived = 0;
        CAN2_LastCRCValid = 0;
    }
    
    // ✅ Cập nhật timestamp
    TimeStamp = HAL_GetTick();
    
    // Print và gửi
    char buffer[15];
    sprintf(buffer, "CAN %u TX\n", (hcan == &hcan1) ? 1 : 2);
    USART3_SendString((unsigned char *)buffer);
    uint8_t id = (hcan == &hcan1) ? 0x12 : 0xA2;
    PrintCANLog(id, data_tx);
    
    if(hcan == &hcan1){
        HAL_CAN_AddTxMessage(hcan, &CAN1_pHeader, data_tx, &CAN1_pTxMailbox);
    }
    else if(hcan == &hcan2){
        HAL_CAN_AddTxMessage(hcan, &CAN2_pHeader, data_tx, &CAN2_pTxMailbox);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    TimeStamp = HAL_GetTick();
    
    if(hcan == &hcan1){
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX);
        char buffer1[15] = "CAN 1 RX\n";
        USART3_SendString((unsigned char *)buffer1);
        PrintCANLog(0xA2, CAN1_DATA_RX);
    }
    else if(hcan == &hcan2){
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_pHeaderRx, CAN2_DATA_RX);
        
        // ✅ Set flag khi nhận được message 0x012
        CAN2_DataReceived = 1;
        
        // ✅ Kiểm tra CRC và set flag
        if(CAN2_DATA_RX[7] == calc_SAE_J1850(CAN2_DATA_RX,7,0)){
            CAN2_LastCRCValid = 1;  // CRC đúng
            char buffer2[15] = "CAN 2 RX\n";
            USART3_SendString((unsigned char *)buffer2);
            PrintCANLog(0x12, CAN2_DATA_RX);
        }
        else{
            CAN2_LastCRCValid = 0;  // CRC sai
            char buffer3[30] = "CAN 2 RX ERROR WRONG CRC\n";
            USART3_SendString((unsigned char *)buffer3);
        }
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t lastButtonTime = 0;
    uint32_t currentTime = HAL_GetTick();
    
    if(GPIO_Pin == GPIO_PIN_0)  // PA0 user button
    {
        // ✅ Debouncing: chỉ accept nếu đã qua 200ms từ lần nhấn trước
        if((currentTime - lastButtonTime) > 200)
        {
            // ✅ Đọc trạng thái pin để confirm button press
            if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
            {
                FlagForBtn = !FlagForBtn;  // Toggle error injection
                
                if(FlagForBtn) {
                    USART3_SendString((uint8_t *)"ERROR INJECTION ON\n");
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);  // LED ON
                } else {
                    USART3_SendString((uint8_t *)"ERROR INJECTION OFF\n");
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);  // LED OFF
                }
                
                lastButtonTime = currentTime;  // Update last button time
            }
        }
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
