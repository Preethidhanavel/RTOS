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
#include<string.h>
#include<stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//macro for chip select pin
#define W25Q_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define W25Q_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
//handle for task
TaskHandle_t sensorHandle;
TaskHandle_t flashHandle;
TaskHandle_t alertHandle;
TaskHandle_t uartHandle;
//handle for queue
QueueHandle_t distance_q;


uint32_t ic_val1 = 0;   // First captured timer value
uint32_t ic_val2 = 0;   // Second captured timer value
uint32_t diff = 0;      // Difference between captures (pulse width)
uint8_t is_first_capture = 0; // Flag to track first/second capture
float distance_cm = 0;     // Calculated distance in centimeters

char msg[64];                   // Message buffer for UART
RTC_TimeTypeDef sTime;          // RTC time structure
RTC_DateTypeDef sDate;          // RTC date structure
char timestamp[32];             // Timestamp string buffer
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
// ================== Read Status Register-1 ==================
static uint8_t W25Qxx_ReadSR1(void)
{
    uint8_t cmd = 0x05; // Command: Read Status Register-1
    uint8_t sr1 = 0;

    W25Q_CS_LOW();                                   // select flash chip
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);          // send command
    HAL_SPI_Receive(&hspi2, &sr1, 1, 100);           // read one byte (status register value)
    W25Q_CS_HIGH();                                  // release flash chip

    return sr1;                                      // return status value
}

// ================== Wait until flash is not busy ==================
static void W25Qxx_WaitBusy(void)
{
    // WIP bit (bit0) = 1 while flash is busy (erase/program in progress)
    while (W25Qxx_ReadSR1() & 0x01)
    {
        osDelay(1); // small delay so other tasks can run
    }
}

// ================== Erase one sector (4KB) ==================
void W25Qxx_EraseSector(uint32_t addr)
{
    uint8_t cmd[4];
    uint8_t we = 0x06;   // Write Enable command

    // --- Send Write Enable ---
    W25Q_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &we, 1, HAL_MAX_DELAY);
    W25Q_CS_HIGH();

    // --- Sector Erase command + 24-bit address ---
    cmd[0] = 0x20;                         // Sector Erase (4KB)
    cmd[1] = (addr >> 16) & 0xFF;          // address byte 2
    cmd[2] = (addr >> 8) & 0xFF;           // address byte 1
    cmd[3] = addr & 0xFF;                  // address byte 0

    W25Q_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);
    W25Q_CS_HIGH();

    W25Qxx_WaitBusy();                     // wait until erase finished
    HAL_Delay(50);                         // extra wait (~50ms typical erase time)
}

// ================== Read flash JEDEC ID ==================
void W25Qxx_Init(void)
{
    uint8_t cmd = 0x9F; // Read JEDEC ID command
    uint8_t id[3];      // buffer for manufacturer/device ID
    char buf[50];

    W25Q_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);  // send command
    HAL_SPI_Receive(&hspi2, id, 3, HAL_MAX_DELAY);     // read 3 ID bytes
    W25Q_CS_HIGH();

    // Print ID over UART (e.g. EF 40 17 for W25Q64)
    sprintf(buf, "W25Q ID: %02X %02X %02X\r\n", id[0], id[1], id[2]);
    HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 1000);
}

// ================== Read data from flash ==================
void W25Qxx_Read(uint32_t addr, uint8_t *buffer, uint16_t size)
{
    uint8_t cmd[4];

    // --- Read Data command + 24-bit address ---
    cmd[0] = 0x03;                         // Read command
    cmd[1] = (addr >> 16) & 0xFF;          // address byte 2
    cmd[2] = (addr >> 8) & 0xFF;           // address byte 1
    cmd[3] = addr & 0xFF;                  // address byte 0

    W25Q_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);   // send command + address
    HAL_SPI_Receive(&hspi2, buffer, size, HAL_MAX_DELAY); // read 'size' bytes into buffer
    W25Q_CS_HIGH();
}

// ================== Write data to flash ==================
void W25Qxx_Write(uint32_t addr, uint8_t *buffer, uint16_t size)
{
    uint8_t cmd[4];
    uint8_t we = 0x06;   // Write Enable command

    // --- Send Write Enable ---
    W25Q_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &we, 1, HAL_MAX_DELAY);
    W25Q_CS_HIGH();

    // --- Page Program command + 24-bit address ---
    cmd[0] = 0x02;                         // Page Program
    cmd[1] = (addr >> 16) & 0xFF;          // address byte 2
    cmd[2] = (addr >> 8) & 0xFF;           // address byte 1
    cmd[3] = addr & 0xFF;                  // address byte 0

    W25Q_CS_LOW();
    HAL_SPI_Transmit(&hspi2, cmd, 4, HAL_MAX_DELAY);   // send command + address
    HAL_SPI_Transmit(&hspi2, buffer, size, HAL_MAX_DELAY); // send data to program
    W25Q_CS_HIGH();                                       // release chip

    W25Qxx_WaitBusy();                     // wait until program finished
}

// ================== Write one float value ==================
void W25Qxx_WriteFloat(uint32_t addr, float value)
{
    uint8_t buffer[4];
    memcpy(buffer, &value, sizeof(float));  // copy float into 4-byte buffer
    W25Qxx_Write(addr, buffer, sizeof(float)); // write 4 bytes to flash
}

// ================== Read one float value ==================
float W25Qxx_ReadFloat(uint32_t addr)
{
    uint8_t buffer[4];
    float value;
    W25Qxx_Read(addr, buffer, sizeof(float)); // read 4 bytes from flash
    memcpy(&value, buffer, sizeof(float));    // copy into float variable
    return value;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function to send a 10µs pulse to TRIG pin
void HCSR04_Trigger(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  //SET TRIG high
 // HAL_Delay(0);  // Wait few cycles
  for (volatile int i = 0; i < 160; i++);  // ~10 µs at 16 MHz
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  //SET TRIG low
}

// Start ultrasonic measurement
void Start_HCSR04(void)
{
  is_first_capture = 0;                                   // Reset capture flag
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);             // Start Input Capture interrupt
  HCSR04_Trigger();                                       // Trigger sensor
}

// Input Capture interrupt callback
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)          // Check channel
  {
    if (is_first_capture == 0) // First edge (rising)
    {
      ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // Read first value
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // Switch to falling edge
      is_first_capture = 1;  // Mark first capture done
    }
    else if (is_first_capture == 1) // Second edge (falling)
    {
      ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // Read second value
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // Switch back to rising
      HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);                   // Stop capture

      // Calculate pulse width
      if (ic_val2 > ic_val1)
        diff = ic_val2 - ic_val1;
      else
        diff = (0xFFFF - ic_val1 + ic_val2);

      // Convert to distance (speed of sound 0.0343 cm/µs, divide by 2 for round trip)
      distance_cm = (float)diff * 0.0343 / 2.0;
    }
  }
}


// ================== Sensor Task ==================
void sensorTask(void *argument)
{
    while(1)
    {
        Start_HCSR04();                          // trigger ultrasonic measurement
   //   HAL_Delay(100);                          // (not used, we rely on vTaskDelay)
        xQueueSend(distance_q, &distance_cm, 0); // send the latest distance value to the queue
        vTaskDelay(pdMS_TO_TICKS(100));          // wait 100 ms before next measurement
    }
}

// ================== Flash Task ==================
void flashTask(void *argument)
{
    float dist, dist1;
    char c[15];
    static uint32_t flashAddr = 0x0001;          // flash write pointer

    while(1)
    {
        // Wait until a distance value is available in the queue
        if(xQueueReceive(distance_q, &dist, portMAX_DELAY))
        {
            W25Qxx_WriteFloat(flashAddr, dist);  // write distance value to flash at current address
            flashAddr += 4;                      // move write pointer

            if(flashAddr >= 0x100000)            // if we reach end of memory (1 MB), wrap back to start
            {
            	W25Qxx_EraseSector(0x0000);     //erase sector
            	flashAddr = 0;
            }

            dist1 = W25Qxx_ReadFloat(flashAddr); // read back value from flash (for verification/debug)

            sprintf(c, "%.2f\r\n", dist1);       // format the read value as string
            HAL_UART_Transmit(&huart2, (uint8_t*)c, strlen(c), 1000); // send value over UART
        }
    }
}

// ================== Alert Task ==================
void alertTask(void *argument)
{
    float dist;

    while(1)
    {
        // Wait until a distance value is available in the queue
        if(xQueueReceive(distance_q, &dist, portMAX_DELAY))
        {
            if(dist < 30)  // threshold: if object is closer than 30 cm
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // turn LED ON (alert)
            else
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // turn LED OFF
        }
    }
}

// UART Task
void uartTask(void *argument)
{
    char msg[64];
    while(1)
    {
    	// Read current RTC time and date
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    	// Format time/date into string
    	sprintf(msg, "Distance: %.2fcm Time: %02d:%02d:%02d | Date: %02d-%02d-20%02d\r\n",distance_cm,sTime.Hours, sTime.Minutes, sTime.Seconds,
    	              sDate.Date, sDate.Month, sDate.Year);
    	//periodic update via UART
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        vTaskDelay(pdMS_TO_TICKS(1000));  //delay
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
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // CS idle HIGH

  /* USER CODE BEGIN 2 */
  uint8_t buffer[6] = {0};                  // buffer to store data read back from flash

  W25Qxx_EraseSector(0x0000);               // erase sector at address 0x0000
  W25Qxx_Init();                            // read and print JEDEC ID of flash chip
  W25Qxx_Write(0x000, (uint8_t *)"Hello", 5); // write 5 bytes ("Hello")
  W25Qxx_Read(0x000, buffer, 5);            // read 5 bytes back from address 0x0000 into buffer
  HAL_UART_Transmit(&huart2, buffer, 5, 1000); // send the read data over UART (should print "Hello")


  if ((RTC->ISR & RTC_ISR_INITS) == 0)
     {
       //RTC not yet initialized. Setting time and date...

       sTime.Hours = 9;
       sTime.Minutes = 30;
       sTime.Seconds = 00;
       sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
       sTime.StoreOperation = RTC_STOREOPERATION_RESET;
       HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

       sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
       sDate.Month = RTC_MONTH_AUGUST;
       sDate.Date = 22;
       sDate.Year = 25;
       HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
     }
  /* USER CODE END 2 */

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  // Create a queue
  distance_q = xQueueCreate(5, sizeof(float));

  // Create SENSOR task , check if creation failed
  if (xTaskCreate(sensorTask, "SENSOR_TASK", configMINIMAL_STACK_SIZE, NULL, 1, &sensorHandle) != pdTRUE)
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)"fail1\r\n", 7, 1000);  // send error message if task creation failed
  }
  // Create FLASH task , check if creation failed
  if (xTaskCreate(flashTask, "FLASH_TASK", configMINIMAL_STACK_SIZE, NULL, 2, &flashHandle) != pdTRUE)
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)"fail2\r\n", 7, 1000);
  }
  // Create ALERT task , check if creation failed
  if (xTaskCreate(alertTask, "ALERT_TASK", configMINIMAL_STACK_SIZE, NULL, 2, &alertHandle) != pdTRUE)
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)"fail3\r\n", 7, 1000);
  }
  // Create UART task , check if creation failed
  if (xTaskCreate(uartTask, "UART_TASK", configMINIMAL_STACK_SIZE, NULL, 2, &uartHandle) != pdTRUE)
  {
      HAL_UART_Transmit(&huart2, (uint8_t*)"fail4\r\n", 7, 1000);
  }



  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
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
