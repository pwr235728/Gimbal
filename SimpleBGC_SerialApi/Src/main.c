/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SBGC.h"
#include "circ_buf.h"
// delay between commands, ms
#define SBGC_CMD_DELAY 20
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GIMBAL_ROT_SPEED 40

#define CIRC_BUFF_LEN 1024


 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
SBGC_ComObj_t com;						// Zawiera adresy funkcji do kuminikacji ze sterownikiem
SBGC_Parser_t parser;					// Przetwarza komendy wychodz¹ca i przychodz¹ce

SBGC_CMD_ControlExt_t ctrlExt;			// Kontrola gimbala

SBGC_SerialCommand_t rx_sc; 			// odebrana komenda
SBGC_RealtimeData_4_t rt_d4; 			// struktura zawierajaca odebrana ramke realtime_data_4

// Buffor ko³owy na odebrane dane
uint8_t _circ_buf_mem[CIRC_BUFF_LEN];
circ_buf_t rx_circ_buf;


// zmienna przechowuj¹ca ostatnio odebrany bajt
uint8_t rx_byte;



// nr osi w tablicy przechowuj¹ce wartoœci z przetwornicka adc (joystick)
#define JOY_X 0
#define JOY_Y 1

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Definicja funkcji do kumunikacji ze sterownikiem
uint16_t getBytesAvailable(void){
	return  circ_buf_bytes_in_buf(&rx_circ_buf);
}

uint8_t readByte(void){
	uint8_t byte;
	circ_buf_read(&rx_circ_buf, &byte);
	return byte;
}

void writeByte(uint8_t b){
	//HAL_UART_Transmit(&huart2, &b, 1, 100);
	HAL_UART_Transmit(&huart1, &b, 1, 100);
}

uint16_t getOutEmptySpace(void){
	return 0xFFFF;
}



// Funkcja odbieraj¹ca dane z uartu
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1){

		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);	// Uruchamia oczekiwanie na kolejny bajt
		circ_buf_write(&rx_circ_buf, rx_byte);  	// wpisuje bajt do buforu ko³owego
	}
}


// zmiana skali wartosci wejsciowej na inna podana
// in_min, in_max - zakres x
// out_min, out_max -  zakresy wyjsciowe
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
 float out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

 out = max(out_min, min(out_max, out)); // range: -1.0f - 1.0f

 return out;
}


// mapuje wartosc z adc  (przetwornik 12bit, zakres 0-4095) na zakres -1.0f do 1.0f z podana martwa strefa
float map_axis_from_adc(uint16_t raw_input, float dead_zone)
{
	float axis = fmap((float)raw_input, 0.0f, 4095.0f, -1.0f, 1.0f);

	// Martwa strefa w srodku
	if(axis < -dead_zone){
		axis = fmap(axis, -1.0f, -dead_zone, -1.0f, 0.0f);
	}else if(axis > dead_zone){
		axis = fmap(axis, dead_zone, 1.0f, 0.0f, 1.0f);
	}else{
		axis = 0;
	}

	return axis;
}


// funkcja przetwarza odebrane dane
// jezeli odebrana zosta³a ramka realtime_data_4 to
// wysyla katy z imu po innym uarcie do komputera
void rx_return_cmd(){


	// dopoki sa w buforze jakies odebrane komendy to je odczytuje
	while (SBGC_Parser_receiveCommand(&parser, &rx_sc)
			== PARSER_STATE_COMPLETE) {
		if (rx_sc.id == SBGC_CMD_CONFIRM) {
			// potwierdzenie ostatniej komendy

			/*uint8_t uart_tx[256];
			uint32_t len = sprintf(uart_tx, "CMD_CONFIRM\r\n");
			HAL_UART_Transmit(&huart2, uart_tx, len, 1000);*/
		}
		if (rx_sc.id == SBGC_CMD_REALTIME_DATA_4) {
			SBGC_cmd_realtime_data_4_unpack(&rt_d4, &rx_sc);

			uint8_t uart_tx[256];
			uint32_t len = sprintf(uart_tx,
					"realtime_data_4 - imu angles: %f, %f, %f \r\n",
					SBGC_ANGLE_TO_DEGREE(rt_d4.cmd_realtime_data_3.imu_angle[ROLL]), // SBGC_ANGLE_TO_DEGREE   jest w pliku SBGC_CommandHelpers.h
					SBGC_ANGLE_TO_DEGREE(rt_d4.cmd_realtime_data_3.imu_angle[PITCH]),
					SBGC_ANGLE_TO_DEGREE(rt_d4.cmd_realtime_data_3.imu_angle[YAW]));
			HAL_UART_Transmit(&huart2, uart_tx, len, 1000);

		}
	}

}

/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t joy_acd[2]; // [X, Y]

	// Inicjalizacja bufora ko³owego
	circ_buf_init(&rx_circ_buf, _circ_buf_mem, CIRC_BUFF_LEN);

	// przypisanie funkcji do komunikacji
	com.getBytesAvailable = getBytesAvailable;
	com.getOutEmptySpace = getOutEmptySpace;
	com.readByte = readByte;
	com.writeByte = writeByte;

	// inicjalizacja parsera
	SBGC_Parser_init(&parser, &com);


	// inicjalizacja struktury kontrolujacej gimbal
	for(int i=0;i<3;i++){
		ctrlExt.mode[i] = SBGC_CONTROL_MODE_SPEED;
		ctrlExt.data[i].angle = 0;
		ctrlExt.data[i].speed = 0;
	}


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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // uruchomienie odbierania danych i przetwornika ADC
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  HAL_ADC_Start_DMA(&hadc2, joy_acd, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {


		float joy_x = -map_axis_from_adc(joy_acd[JOY_X], 0.1f);
		joy_x *= GIMBAL_ROT_SPEED;
		ctrlExt.data[YAW].speed = joy_x * (SBGC_SPEED_SCALE);

		float joy_y = -map_axis_from_adc(joy_acd[JOY_Y], 0.1f);
		joy_y *= GIMBAL_ROT_SPEED;
		ctrlExt.data[PITCH].speed = joy_y * (SBGC_SPEED_SCALE);


		// wyslanie komendy sterujacej gimbalem
		SBGC_cmd_control_ext_send(&ctrlExt, &parser);
		HAL_Delay(30);		// odczekanie az dane zostana odebrane
		rx_return_cmd();	// odebranie danych

		// wyslanie zapytania o realtime_data_4
		SBGC_cmd_control_rtData4_send(&parser);
		HAL_Delay(30); 		// odczekanie az dane zostana odebrane
		rx_return_cmd(); 	// odebranie danych


		// komendy mozna wysylac kilka naraz a pozniej odebrac dane na koncu np:
		/*
		 *
		 SBGC_cmd_control_ext_send(&ctrlExt, &parser);
		 SBGC_cmd_control_rtData4_send(&parser);
		 .
		 .
		 .
		 rx_return_cmd();

		 */

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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_LEFT_Pin BTN_RIGHT_Pin BTN_UP_Pin BTN_DOWN_Pin */
  GPIO_InitStruct.Pin = BTN_LEFT_Pin|BTN_RIGHT_Pin|BTN_UP_Pin|BTN_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
