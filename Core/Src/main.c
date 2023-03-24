/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "limits.h"
#include "stdbool.h"
#include "NRF24L01_macros.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define inputSize (160) // wave your hands in the air like you don't care
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define latch()   HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);\
  	HAL_Delay(1);\
  	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t regD= 0b00000110; //состояние сдвиговика
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void NRF_Transmit(uint8_t *data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RF_ShiftRegs(){
			HAL_GPIO_WritePin(RF_SRE_GPIO_Port, RF_SRE_Pin, 1); //shiftreg 2 HiZ
			uint8_t buf;
			HAL_GPIO_WritePin(RF_SRL_GPIO_Port, RF_SRL_Pin, 0);  //отключаем защёлку

			HAL_SPI_TransmitReceive(&hspi2, &regD, &buf, 1, 5000);  //отправляем данные для сдвигового регистра со светодиодами

			HAL_GPIO_WritePin(RF_SRL_GPIO_Port, RF_SRL_Pin, 1);  //включаем защёлку
			HAL_GPIO_WritePin(RF_SRE_GPIO_Port, RF_SRE_Pin, 0);  //shitfreg вывести из HiZ
}

void NRF_cs(uint8_t flag){
		if(flag==0)
			{
			regD &=~(1<<1);
			//HAL_GPIO_WritePin(NRF_CS_GPIO_Port,NRF_CS_Pin,0);
			}
		else
			{
			regD |= 1<<1;
			//HAL_GPIO_WritePin(NRF_CS_GPIO_Port,NRF_CS_Pin,1);
			}
		RF_ShiftRegs();
}

void NRF_ce(uint8_t flag){
	/*
		 * Данная функция нужна для управления ножкой CE NRF
		 * Если на вход подаётся 0, то на ножке появляется ноль, если другое число, то единица
		 */

		if(flag==0)
		{
			regD &=~(1<<0);
			//HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,0);
		}
		else
			{
			regD |= 1<<0;
			//HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,1);
			}
		RF_ShiftRegs();
}

void NRF_write_reg(uint8_t reg, uint8_t val){
	NRF_cs(0);

	reg |= NRF_WRITE_MASK;
	uint8_t buf;
	HAL_SPI_TransmitReceive(&hspi2, &reg, &buf, sizeof(reg), 10000);
	HAL_SPI_TransmitReceive(&hspi2, &val, &buf, sizeof(val), 10000);

	NRF_cs(1);
}

uint8_t NRF_read_reg(uint8_t reg){
	uint8_t rx_data;
	NRF_cs(0);

	HAL_SPI_TransmitReceive(&hspi2, &reg, &rx_data, 1, 10000);
	if(reg != NRF_REG_STATUS)
		{
			reg=NOP;

			HAL_SPI_TransmitReceive(&hspi2, &reg, &rx_data, 1, 500);
		}

	NRF_cs(1);

	return rx_data;
}

void NRF_burst_write(uint8_t reg, uint8_t *data, uint8_t len){// not used


	NRF_cs(0);

	uint8_t buf;
	uint8_t tmp = NRF_WRITE_MASK | reg;
	HAL_SPI_TransmitReceive(&hspi2, &tmp, &buf, sizeof(tmp), 10000);
	HAL_SPI_TransmitReceive(&hspi2, data, &buf, sizeof(data), 10000);

	NRF_cs(1);
}

void NRF_initialization(){
HAL_Delay(150); //задержка включения
uint8_t NRF_TX_ADDR[5] = {0x01,0x02,0x03,0x04,0x05}; // {0xff,0x00,0x00,0x00,0xff}; //address of transmitter
uint8_t NRF_RX_ADDR[5] = {0x01,0x02,0x03,0x04,0x05} /*= {0xff,0x00,0x00,0x00,0xff} = {0xD1,0xD2,0xD3,0xD4,0xD5}*/; // Адрес приёмника

//////////////////////////////////////////////////////////// Start Initialization
 uint8_t FANTOM_CONFIG = NRF_REG_CONFIG_PWR_UP | NRF_REG_CONFIG_CRCO | NRF_REG_CONFIG_EN_CRC; //присваиваем переменной значение бита PWR_UP в 1 и выбираем CRC в 2 байта и включение режима приёмника.
NRF_write_reg(NRF_REG_CONFIG, FANTOM_CONFIG); // go to Standby-I mode

HAL_Delay(200);
//Сейчас мы в Standby-I mode
//FANTOM_CONFIG = FANTOM_CONFIG & (~(NRF_REG_CONFIG_EN_CRC)); // присваиваем переменной значение бита EN_CRC в 0
///////////////////////////////////////////////////////////// Запись в регистр CONFIG SPDR = 0b00100110;
//NRF_write_reg(NRF_REG_CONFIG, FANTOM_CONFIG); //нафиг
HAL_Delay(2);
/////////////////////////////////////////////////////////////Disable fucking EN_AA
NRF_write_reg(NRF_REG_EN_AA, 0x00);
/////////////////////////////////////////////////////////////Выбор pipe0
NRF_write_reg(NRF_REG_EN_RXADDR, NRF_REG_EN_RXADDR_ERX_P0);
///////////////////////////////////////////////////////////// Настраиваем частоту волны на 2400 МГЦ
NRF_write_reg(NRF_REG_RF_CH, 0x00);
////////////////////////////////////////////////////////////Настраиваем длину адреса на 5 байт
NRF_write_reg(NRF_REG_SETUP_AW, NRF_REG_SETUP_AW_AW0 | NRF_REG_SETUP_AW_AW1);
//////////////////////////////////////////////////////////// Настраиваем приёмник 0b00100110
NRF_write_reg(NRF_REG_RSETUP_AW, NRF_REG_RSETUP_AW_RF_RF_DR_LOW //скорость 250kbts
|NRF_REG_RSETUP_AW_RF_PWR1|NRF_REG_RSETUP_AW_RF_PWR0);// мощность 0dBm
/////////////////////////////////////////////////////////// Настройка длины пакета в RX_PW_P0 в pipe 0 на 32 байта
NRF_write_reg(NRF_REG_RX_PW_P0, NRF_PACKET_SIZE);
///////////////////////////////////////////////////////////Clear Interupt flag in Status
NRF_write_reg(NRF_REG_STATUS, NRF_REG_STATUS_TX_DS
|NRF_REG_STATUS_RX_DR
|NRF_REG_STATUS_MAX_RT); //Нет в файле "Салюта"

/////////////////////////////////////////////////////////// Настраиваем адрес приёмника RX_ADDR_P0 - using default address E7E7E7E7E7
//NRF_burst_write(NRF_REG_RX_ADDR_P0, NRF_RX_ADDR, 5);
///////////////////////////////////////////////////////////Настраиваем адрес передатчика TX_ADDR
//NRF_burst_write(NRF_REG_TX_ADDR, NRF_TX_ADDR, 5);
/////////////////////////////////////////////////////////////Disable fucking dynamic payload
//NRF_write_reg(NRF_DYNPD, 0x00);
/*/*********************************************************Disable fucking dynamic payload*/
//NRF_write_reg(NRF_FEATURE, 0x00);
HAL_Delay(5);
}

void NRF_Transmit(uint8_t *data){
	uint8_t a = 0;
	NRF_ce(0);
	HAL_Delay(1);
#warning missing delay - we use 1 ms instead of 15 microsecs.
	NRF_cs(0);
	uint8_t tmp_com = NRF24_COMMAND_W_TX_PAYLOAD;
	HAL_SPI_Transmit(&hspi2, &tmp_com, sizeof(tmp_com), 1);
	for(uint8_t i = 0; i < 32; i++){
		HAL_SPI_Transmit(&hspi2, &data[i], 1, 1);
#warning hardcode array size - 32.
	}

	HAL_Delay(1);
#warning missing delay - we use 1 ms instead of 2 microsecs.
	NRF_cs(1);
	NRF_ce(1);
	HAL_Delay(1);
#warning missing delay - we use 1 ms instead of 130 microsecs.
	NRF_ce(0);

	while(NRF_read_reg(NRF_REG_FIFO_STATUS) & NRF_REG_FIFO_STATUS_TX_EMPTY){
		a++;
		HAL_Delay(1);
		if(a >= 100) break;
	}


}

char parseResult[] = "\nlatitude is .. ........, longitude is ... ........, height is ......... M\r\n";
char permParseResult[] = "\nlatitude is .. ........, longitude is ... ........, height is ......... M\r\n";

int parseGNSS(uint8_t input[inputSize]){ // we want to find $GPGGA because it contains height, then parse it to spit out the longitude, latitude, height.
	int64_t start = INT_MAX - 1;
	int64_t temp_iter;
	char currentProtocol[6];
	bool identical = true;
	for (int i = 0; i < inputSize; i++){
		if (input[i] == '$'){
			for (int j = i; (j < (i + 6)) && (j < inputSize); j++){
				currentProtocol[j-i] = input[j];
				temp_iter = j;
			}
			if (temp_iter >= inputSize) printf("\nmoved over the edge of the input, trying to find the $GPwhatever, j >= inputSize\r\n");
			for (int j = 0; j < 6; j++){
				if (currentProtocol[j] != "$GPGGA"[j]) identical = false;
			}
			if (identical) {
				start = i;
				break;
			}
			else identical = true;
		}
	}
	// parsing
	if (start == INT_MAX - 1){
		printf("\nparsing failed, start == INT_MAX - 1\r\n");
		return 10;
	}
	int64_t pointer = start;
	while (input[pointer] != ',' && (pointer < inputSize)) {pointer++;} // skip $GPGGA,
	pointer++;
	while (input[pointer] != ',' && (pointer < inputSize)) {pointer++;} // skip time,
	pointer++;

	if (input[pointer] == ',') {} // latitude is missing
	else { // latitude
		parseResult[13] = input[pointer++]; // this is bad, because no && (pointer < inputSize), but ok
		parseResult[14] = input[pointer++];
		parseResult[16] = input[pointer++];
		parseResult[17] = input[pointer++];
		parseResult[18] = input[pointer++];
		parseResult[19] = input[pointer++];
		parseResult[20] = input[pointer++];
		parseResult[21] = input[pointer++];
		parseResult[22] = input[pointer++];
	}
	pointer++;
	//printf("%s\n\n", parseResult); //debug
	if (input[pointer] != ',') { // make sure that north/south isn't missing
		parseResult[23] = input[pointer];
	}
	//printf("%s\n\n", parseResult);//debug
	pointer += 2;

	if (input[pointer] == ',') {} // longitude is missing
	else { // latitude
		parseResult[39] = input[pointer++];
		parseResult[40] = input[pointer++];
		parseResult[41] = input[pointer++];
		parseResult[43] = input[pointer++];
		parseResult[44] = input[pointer++];
		parseResult[45] = input[pointer++];
		parseResult[46] = input[pointer++];
		parseResult[47] = input[pointer++];
		parseResult[48] = input[pointer++];
		parseResult[49] = input[pointer++];
	}
	pointer++;
	if (input[pointer] != ',') { // make sure that hemisphere isn't missing
		parseResult[50] = input[pointer++];
	}
	pointer++;

	while (input[pointer] != ',' && (pointer < inputSize)) {pointer++;} // skip mode,
	pointer++;
	while (input[pointer] != ',' && (pointer < inputSize)) {pointer++;} // skip number of used sattelites,
	pointer++;
	while (input[pointer] != ',' && (pointer < inputSize)) {pointer++;} // skip HDOP, whatever it is
	pointer++;
	// height
	for (int i = pointer; ((i - pointer) < 9) && (i < inputSize); i++){ // pointer+1 because I wanna to put ">" in there if there are too many symbols.
		if (input[i] == ',') break;
		parseResult[63 + i - pointer] = input[i];
		//printf(" i is %u input[i] is %c\r\n", i, input[i]); //debug
	}

	if (pointer >= inputSize){
		printf("\nparsing failed, pointer >= inputSize\r\n");
		return 20;
	}

	return 0;
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
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//HAL_GPIO_WritePin(nOE_GPIO_Port, nOE_Pin, GPIO_PIN_RESET); //НЕ говорить с nrf
	/*regD = 0xFE; //NRF24_CE_PIN = 0, NRF24_CS_PIN = 1
  	HAL_SPI_Transmit(&hspi2, &regD, 1, 10000);
  	latch();*/
  NRF_initialization();
  uint8_t input[inputSize];
  //NRF_write_reg(NRF_REG_RF_CH, NRF_CHANEL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

	  //HAL_Delay(3000);
	  uint8_t str[100];
	  uint8_t hello_str[32] = "Hello";
	  /*for(uint8_t i=0;i<0x1D;i++){
		  uint8_t current_value = NRF_read_reg(i);
		  snprintf(str, sizeof(str), "adr%X = %X ", i, current_value);
		  CDC_Transmit_FS(str, strlen(str));
		  HAL_Delay(300);
		  NRF_Transmit(hello_str);
	  }*/
	  HAL_UART_Receive(&huart2, input, sizeof(input)/sizeof(input[0]), 1000);
	  int j = parseGNSS(input);
	  printf("%s\n\n", parseResult);
	  NRF_Transmit(parseResult);
	  for (int i = 0; i < sizeof(permParseResult)/sizeof(permParseResult[0]); i++){
		  parseResult[i] = permParseResult[i];
	  }
	  HAL_Delay(1000);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nOE_GPIO_Port, nOE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LATCH_Pin nOE_Pin */
  GPIO_InitStruct.Pin = LATCH_Pin|nOE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int __io_getchar(void){
	uint8_t data;
	HAL_UART_Receive(&huart2, &data, sizeof(data), 1000);
	return data;
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
