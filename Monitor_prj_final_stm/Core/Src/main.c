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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint16_t temp_raw;
	uint16_t humid_raw;
	uint8_t humid;
	float temp;
}data_t;

//GPS_data
typedef struct{
	char lat[20];
	char lon[20];
	char time[12];
	char date[12];
	float speed;
}GPS_data;


typedef enum{
	DHT11_DATA,
	MPU6050_DATA,
	MQ2_DATA
}Sensor_data_type;

typedef struct{
	Sensor_data_type type;
	union{
		data_t data_dht;
		MPU_Data_t data_mpu;
	}data;
	uint32_t mq2_data;

}Sensor_data_t;

typedef struct{
	data_t data_dht;
	MPU_Data_t data_mpu;
	uint32_t mq2_data;
	GPS_data gps_data;
}All_Sensor_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define STACK_DEPTH 2048UL
#define DHT_TASK_PRIORITY tskIDLE_PRIORITY+1
#define MQ2_TASK_PRIORITY tskIDLE_PRIORITY+2
#define TRANSMIT_TASK_PRIORITY tskIDLE_PRIORITY+3
#define MPU_TASK_PRIORITY tskIDLE_PRIORITY+4
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart5_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
//static char readings[200] = {0};
TaskHandle_t dht_task,mpu_task,transmit_task,mq2_task,sensor_data_aquire;
QueueHandle_t sensor_queue;
dht11_handle_t dht_handle = {0};
BaseType_t res;
char dht_readings[100];
char mq2_buff[50];
char sens_data_buff[500];
char transmit_buff[550];
char recv_buff[200];
char *debug = "Task not created\r\n";
char err_msg[50] = "error Occurred\n\r";
char crc_value[100] = {0};
float lat_raw,lon_raw,speed_raw,course_raw,time_raw;
uint32_t date_raw;
char dir1,dir2,mode;
char gps_check_sum[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART5_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void DHT11_Task(void *params);
void MPU_6050_Task(void *params);
void Transmit(void *params);
void Sensor_data_aquire(void *params);
static void DHT11_Sensor_Init(void);
static void task_create(void);
static uint8_t gps_parse_data(All_Sensor_data *sensor_data);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Transmit(&huart2, (uint8_t*)debug, strlen(debug), HAL_MAX_DELAY);
  //lcd_init();
  HAL_UART_Receive_DMA(&huart5, (uint8_t*)recv_buff, sizeof(recv_buff));
  mpu6050_init();
  DHT11_Sensor_Init();
  crcInit();
  HAL_UART_Transmit(&huart2, (uint8_t*)"Peripherals Initialized", 30, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  sensor_queue = xQueueCreate(10, sizeof(All_Sensor_data));
  if(sensor_queue == NULL){
	  Error_Handler();
  }

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  task_create();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA2_Channel2_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void DHT11_Sensor_Init(void){

	 //Enable the DHT interface
		/* link interface function */
	    DRIVER_DHT11_LINK_INIT(&dht_handle, dht11_handle_t);
	    DRIVER_DHT11_LINK_BUS_INIT(&dht_handle, dht11_interface_init);
	    DRIVER_DHT11_LINK_BUS_DEINIT(&dht_handle, dht11_interface_deinit);
	    DRIVER_DHT11_LINK_BUS_READ(&dht_handle, dht11_interface_read);
	    DRIVER_DHT11_LINK_BUS_WRITE(&dht_handle, dht11_interface_write);
	    DRIVER_DHT11_LINK_DELAY_MS(&dht_handle, dht11_interface_delay_ms);
	    DRIVER_DHT11_LINK_DELAY_US(&dht_handle, dht11_interface_delay_us);
	    DRIVER_DHT11_LINK_ENABLE_IRQ(&dht_handle, dht11_interface_enable_irq);
	    DRIVER_DHT11_LINK_DISABLE_IRQ(&dht_handle, dht11_interface_disable_irq);
	    DRIVER_DHT11_LINK_DEBUG_PRINT(&dht_handle, dht11_interface_debug_print);

	    //Initialize the dht
	    uint8_t res = dht11_init(&dht_handle);
	    if(res != 0){
			dht_handle.debug_print("Initialization Error\n");
		}
}

static uint8_t gps_parse_data(All_Sensor_data *sensor_data){
	char data_buff[100] = {0};
	uint8_t i = 0, j = 0;

	while (recv_buff[i] != '$' && i < sizeof(recv_buff)) i++;
	if (i >= sizeof(recv_buff)) return 2;

	while (recv_buff[i] != '\n' && j < sizeof(data_buff) - 1 && i < sizeof(recv_buff)) {
		data_buff[j++] = recv_buff[i++];
	}
	data_buff[j++] = '\n';
	data_buff[j] = '\0';
	//HAL_UART_Transmit(&huart2, (uint8_t*)data_buff, strlen(data_buff), HAL_MAX_DELAY);
	// Send it out (for debug/verification)
	//HAL_UART_Transmit(&huart2, (uint8_t *)data_buff, strlen(data_buff), HAL_MAX_DELAY);
	uint8_t res = sscanf(data_buff,"$GNRMC,%f,%c,%f,%c,%f,%c,%f,%f,%ld,,,%s\r\n",&time_raw,&mode,&lat_raw,&dir1,&lon_raw,&dir2,&speed_raw,&course_raw,&date_raw,gps_check_sum);
	if(mode != 'A' || res!=10){
		return 1;
	}
	else{
	//Parse Time and Date in Local Time -> UTC + 5hrs,30min Format
	uint8_t hr = time_raw/10000;
	uint8_t min = (time_raw - 10000*hr)/100;
	uint8_t sec = (uint8_t)fmod(time_raw,100);
	uint8_t day = date_raw/10000;
	uint8_t month = (date_raw - 10000*day)/100;
	uint8_t year = (uint8_t)fmod(date_raw,100);
	uint8_t days_in_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
	if(year%4 == 0){
		days_in_month[1] = 29;
	}
	hr+=5;
	min+=30;
	if(min>=60){
		min-=60;
		hr+=1;
	}
	if(hr>=24){
		hr = hr - 24;
		day+=1;
	}
	if(day>days_in_month[month - 1]){
		day = 1;
		month += 1;
	}
	if(month > 12){
		month = 1;
		year +=1;
		if(year>99){
			year = 0;
		}
	}
	sprintf(sensor_data->gps_data.time,"%02d:%02d:%02d",hr,min,sec);
	sprintf(sensor_data->gps_data.date,"%02d:%02d:%02d",day,month,year);

	//Parse Location
	uint16_t degree_lat = lat_raw/100;
	float minute_lat = (lat_raw - degree_lat*100);
	float final_lat = degree_lat + (minute_lat/60);
	uint16_t degree_lon = lon_raw/100;
	float minute_lon = (lon_raw - degree_lon*100);
	float final_lon = degree_lon + (minute_lon/60);
	sprintf(sensor_data->gps_data.lat,"%f%c",final_lat,dir1);
	sprintf(sensor_data->gps_data.lon,"%f%c",final_lon,dir2);

	//Parse Speed
	float speed = speed_raw*1.852;//Knots to kmph
	sensor_data->gps_data.speed = speed;

	return 0;
	}

}

#if (CRC_CHECK_ENABLED == 0)

void DHT11_Task(void *pvParameters){
	while(1){
		Sensor_data_t sensor_data;
		sensor_data.type = DHT11_DATA;
		uint8_t res = dht11_read_temperature_humidity(&dht_handle, &sensor_data.data.data_dht.temp_raw, &sensor_data.data.data_dht.temp, &sensor_data.data.data_dht.humid_raw, &sensor_data.data.data_dht.humid);
		if(res == 0){
		xQueueSend(sensor_queue,&sensor_data,portMAX_DELAY);
		}
		vTaskDelay(pdMS_TO_TICKS(1400));//Wait for 2s
	}
}


void MPU_6050_Task(void *pvParameters){
	while(1){
		Sensor_data_t sensor_data;
		sensor_data.type = MPU6050_DATA;
		sensor_data.data.data_mpu= MPU6050_read_all();
		xQueueSend(sensor_queue,&sensor_data,portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(1000));//Wait for 500ms
	}

}

void MQ2_Task(void *pvParameters){
	while(1){
	  Sensor_data_t sensor_data;
	  sensor_data.type = MQ2_DATA;
	  HAL_ADC_Start(&hadc1);
	  while(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK);
	  sensor_data.mq2_data = HAL_ADC_GetValue(&hadc1);
	  xQueueSend(sensor_queue,&sensor_data,portMAX_DELAY);
	  vTaskDelay(1200);//Wait for 1s
	}//Wait for

}


void Transmit(void *pvParameters){
	Sensor_data_t sensor_data;
	while(1){
		if(xQueueReceive(sensor_queue, &sensor_data, portMAX_DELAY)){
			switch(sensor_data.type){
			case DHT11_DATA:
				sprintf(dht_readings,"A:Temperature is %.2f and Humidity is %.2d\r\n", sensor_data.data.data_dht.temp,sensor_data.data.data_dht.humid);
				HAL_UART_Transmit(&huart1, (uint8_t*)dht_readings, strlen(dht_readings), HAL_MAX_DELAY);
				break;
			case MPU6050_DATA:
				sprintf(readings,"B:Acceleration ax=%.2f , ay =%.2f , az = %.2f\n\r Angular Velocity wx = %.2f , wy = %.2f , wz = %.2f\r\n",sensor_data.data.data_mpu.acc_x,sensor_data.data.data_mpu.acc_y,sensor_data.data.data_mpu.acc_z,sensor_data.data.data_mpu.gyr_x,sensor_data.data.data_mpu.gyr_y,sensor_data.data.data_mpu.gyr_z);
				HAL_UART_Transmit(&huart1,(uint8_t*)readings, strlen(readings), HAL_MAX_DELAY);
				break;
			case MQ2_DATA:
				sprintf(mq2_buff,"C:PPM concentration is %lu\n\r",sensor_data.mq2_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)mq2_buff, strlen(mq2_buff), HAL_MAX_DELAY);
			}
		}
	}

}

#else

void Sensor_data_aquire(void *pvParameters){
	while(1){
		All_Sensor_data sens_data;
		uint8_t res = dht11_read_temperature_humidity(&dht_handle, &sens_data.data_dht.temp_raw,&sens_data.data_dht.temp,&sens_data.data_dht.humid_raw,&sens_data.data_dht.humid);
		//HAL_Delay(100);
		if(res != 0){
			//HAL_UART_Transmit(&huart2, (uint8_t*)"DHT Error Occurred\r\n", 30, HAL_MAX_DELAY);
		}
		sens_data.data_mpu = MPU6050_read_all();
		HAL_ADC_Start(&hadc1);
		while(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK);
		sens_data.mq2_data = HAL_ADC_GetValue(&hadc1);
		uint8_t rec = gps_parse_data(&sens_data);
		if(rec!=0){
			//HAL_UART_Transmit(&huart2, (uint8_t*)"GPS Error Occurred\r\n", 30, HAL_MAX_DELAY);
		}
		xQueueSend(sensor_queue,&sens_data,portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(3000));
	}
}

void Transmit(void *pvParameters){
	while(1){
		All_Sensor_data sens_data;
		if(xQueueReceive(sensor_queue, &sens_data, portMAX_DELAY)){
			sprintf(sens_data_buff,"%f,%u,%f,%f,%f,%f,%f,%f,%lu,%s,%s,%s,%s,%f",\
			sens_data.data_dht.temp,sens_data.data_dht.humid,sens_data.data_mpu.acc_x,\
			sens_data.data_mpu.acc_y,sens_data.data_mpu.acc_z,sens_data.data_mpu.gyr_x,\
			sens_data.data_mpu.gyr_y,sens_data.data_mpu.gyr_z,sens_data.mq2_data,sens_data.gps_data.lat,sens_data.gps_data.lon,\
			sens_data.gps_data.date,sens_data.gps_data.time,sens_data.gps_data.speed);
			crc crc_val = crcFast((unsigned char*)sens_data_buff, strlen(sens_data_buff));
			//sprintf(crc_value,"%lu %d %.2f\n\r",crc_val,sens_data.data_dht.humid,sens_data.data_dht.temp);
			//HAL_UART_Transmit(&huart2, (uint8_t*)&crc_value, 100, HAL_MAX_DELAY);
			sprintf(transmit_buff,"AA,%s,%lu\r\n",sens_data_buff,crc_val);
			HAL_UART_Transmit(&huart1, (uint8_t*)transmit_buff, strlen(transmit_buff), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2,  (uint8_t*)transmit_buff, strlen(transmit_buff), HAL_MAX_DELAY);
		}
	}
}
#endif


static void task_create(void){
#if(CRC_CHECK_ENABLED == 0)
	res = xTaskCreate(DHT11_Task, "DHT11 Task", STACK_DEPTH, NULL, DHT_TASK_PRIORITY, &dht_task);
	  if(res != pdPASS){
		  HAL_UART_Transmit(&huart2, (uint8_t*)"DHT task creation failed\n", 30, HAL_MAX_DELAY);
	  }
	  //configASSERT( xHandle );
	  res = xTaskCreate(MPU_6050_Task, "MPU6050 Task", STACK_DEPTH, NULL, MPU_TASK_PRIORITY, &mpu_task);
	  if(res != pdPASS){
		  HAL_UART_Transmit(&huart2, (uint8_t*)"MPU task creation failed\n", 30, HAL_MAX_DELAY);
	    }
	  res = xTaskCreate(Transmit,"UART Transmit",STACK_DEPTH,NULL,TRANSMIT_TASK_PRIORITY,&transmit_task);
	  if(res != pdPASS){
		  HAL_UART_Transmit(&huart2, (uint8_t*)"Transmit task creation failed\n", 30, HAL_MAX_DELAY);
	    }
	  res = xTaskCreate(MQ2_Task, "MQ2 Task", STACK_DEPTH, NULL, MQ2_TASK_PRIORITY, &mq2_task);
	  if(res != pdPASS){
		  HAL_UART_Transmit(&huart2, (uint8_t*)"MQ2 task creation failed\n", 30, HAL_MAX_DELAY);
	  }
#else
	  res = xTaskCreate(Sensor_data_aquire, "Sensor_Data_Aquire", STACK_DEPTH, NULL, 1, &sensor_data_aquire);
	  if(res != pdPASS){
		  HAL_UART_Transmit(&huart2, (uint8_t*)"Data Aquire task creation failed\n", 30, HAL_MAX_DELAY);
	  }
	  res = xTaskCreate(Transmit,"UART Transmit",STACK_DEPTH,NULL,2,&transmit_task);
	  if(res != pdPASS){
		  HAL_UART_Transmit(&huart2, (uint8_t*)"Transmit task creation failed\n", 30, HAL_MAX_DELAY);
	  }
#endif


}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

#ifdef USE_FULL_ASSERT
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
