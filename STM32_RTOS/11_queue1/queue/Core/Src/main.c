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
#include<stdio.h>
#include<string.h>
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
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

// Declare task handles
TaskHandle_t taskhandle1 =NULL;
TaskHandle_t taskhandle2 =NULL;
TaskHandle_t taskhandle3 =NULL;
TaskHandle_t taskhandle4 =NULL;

// Declare queue handles
QueueHandle_t cmd_q = NULL;   // Queue for command messages
QueueHandle_t uart_q = NULL;  // Queue for UART messages

// Define a command structure
typedef struct {
    uint8_t cmd_no;       // Command number
    uint8_t cmd_arg[10];  // Arguments for the command
} cmd_t;

// Buffer to store command input from UART
uint8_t cmd_buf[20];
uint8_t cmd_len = 0;

// Menu string that will be sent to UART for user
char menu[]="\r\nLED_ON ->1\r\nLED_OFF ->2\r\nLED_TOGGLE ->3\
		\r\nLED_READ_STATUS ->4\r\nRTC_PRINT ->5\
		\r\nEXIT ->0\r\nENTER YOUR OPTION:";

// Define command macros for easy reference
#define LED_ON 	1
#define	LED_OFF 2
#define LED_TOGGLE 3
#define LED_READ_STATUS 4
#define RTC_READ 5
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
uint8_t getCommandCode(uint8_t *buffer);

//prototypes command helper functions
void make_led_on(void);
void make_led_off(void);
void led_toggle(void);
void read_led_status(char *task_msg);
void read_rtc_info(char *task_msg);
void print_error_message(char *task_msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t rx_byte;

// UART receive complete interrupt callback (called when a byte is received)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART2)  // Ensure it is USART2
    {
        // Echo received byte back using interrupt transmit
        HAL_UART_Transmit_IT(&huart2, &rx_byte, 1);

        // Store received byte into buffer
        cmd_buf[cmd_len++] = rx_byte;

        // If ENTER key (\r) is pressed, command is complete
        if (rx_byte == '\r')
        {
            cmd_len = 0; // reset buffer index for next command

            // Notify tasks 1 and 2 that command is ready
            vTaskNotifyGiveFromISR(taskhandle1, &xHigherPriorityTaskWoken);
            vTaskNotifyGiveFromISR(taskhandle2, &xHigherPriorityTaskWoken);
        }

        // Re-enable UART to receive next byte
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

        // Yield to higher priority task if required
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Simple wrapper to send string over UART (blocking mode)
void uart_send(char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
}

// Task 1: Sends the menu to UART queue and waits for notification
void task1_menu(void *p)
{
    char *data = menu;   // Menu string to be sent

    while(1)
    {
        // Put menu string in UART queue
        xQueueSend(uart_q, &data, portMAX_DELAY);

        // Wait until notified (when new command arrives)
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    }
}

// Task 2: Parses received command and forwards it for processing
void task2_command(void *p)
{
    uint8_t cmd_code = 0;
    cmd_t *new_cmd;

    while(1)
    {
        // Wait until notified (command received)
        xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

        // Allocate memory for new command structure
        new_cmd = (cmd_t*)pvPortMalloc(sizeof(cmd_t));

        // Critical section: read and store command code
        taskENTER_CRITICAL();
        cmd_code = getCommandCode(cmd_buf);
        new_cmd->cmd_no = cmd_code;
        // getArguments(new_cmd->cmd_arg);  // Placeholder if arguments exist
        taskEXIT_CRITICAL();

        // Send new command structure to command queue
        xQueueSend(cmd_q, &new_cmd, portMAX_DELAY);
    }
}

void task3_command_process(void *p)
{
    cmd_t *new_cmd;         // Pointer to hold received command
    char task_msg[50];      // Buffer to store messages (like LED/RTC info)

    while(1)                // Infinite loop (task runs forever)
    {
        // Wait until a command is received from the queue (blocking)
        xQueueReceive(cmd_q, (void*)&new_cmd, portMAX_DELAY);

        // Process command based on cmd_no
        if(new_cmd->cmd_no == LED_ON)
        {
            make_led_on();  // Turn LED ON
        }
        else if(new_cmd->cmd_no == LED_OFF)
        {
            make_led_off(); // Turn LED OFF
        }
        else if(new_cmd->cmd_no == LED_TOGGLE)
        {
            led_toggle();   // Toggle LED ON/OFF
        }
        else if(new_cmd->cmd_no == LED_READ_STATUS)
        {
            read_led_status(task_msg); // Read LED status into task_msg
        }
        else if(new_cmd->cmd_no == RTC_READ)
        {
            read_rtc_info(task_msg);   // Read RTC info into task_msg
        }
        else
        {
            print_error_message(task_msg); // Handle invalid command
        }

        // Free the dynamically allocated command memory
        vPortFree(new_cmd);
    }
}

// Task 4: Transmits data placed in UART queue
void task4_uart_transmit(void *p)
{
    char *data = NULL;

    while(1)
    {
        // Wait for data from UART queue
        xQueueReceive(uart_q, &data, portMAX_DELAY);

        // Send the data string over UART
        uart_send(data);
    }
}

// Extracts command code from buffer
uint8_t getCommandCode(uint8_t *buffer)
{
    return buffer[0] - 48;  // Convert ASCII to number
}


// Turns LED ON (PA5)
void make_led_on(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
}

// Turns LED OFF (PA5)
void make_led_off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
}

void led_toggle(void)
{
    // Toggle LED 20 times with 500ms delay
    for(uint8_t i = 0; i < 20; i++)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);   // Toggle LED on pin PA5
        vTaskDelay(pdMS_TO_TICKS(500));          // Delay 500 ms (FreeRTOS delay)
    }
}
//Read LED status
void read_led_status(char *task_msg)
{
	sprintf(task_msg , "\r\nLED status is : %d\r\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));
	xQueueSend(uart_q,&task_msg,portMAX_DELAY);//write the LED status to Queue
}

// Reads RTC time/date and sends to UART queue
void read_rtc_info(char *task_msg)
{
    RTC_TimeTypeDef sTime = {0};  // Structure to store RTC time
    RTC_DateTypeDef sDate = {0};  // Structure to store RTC date
    if ((RTC->ISR & RTC_ISR_INITS) == 0)
      {
        //RTC not yet initialized. Setting time and date...
       // Set default time: 11:15:30
        sTime.Hours = 9;
        sTime.Minutes = 15;
        sTime.Seconds = 30;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        // Set default date: Friday, 29-Aug-2025
        sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
        sDate.Month = RTC_MONTH_AUGUST;
        sDate.Date = 20;
        sDate.Year = 25;
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
      }

    // Read current RTC time and date
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Format time/date into string
    sprintf(task_msg, "Time: %02d:%02d:%02d | Date: %02d-%02d-20%02d\r\n",
              sTime.Hours, sTime.Minutes, sTime.Seconds,
              sDate.Date, sDate.Month, sDate.Year);

    // Send formatted message to UART queue
    xQueueSend(uart_q, &task_msg, portMAX_DELAY);
}

// Sends error message to UART queue
void print_error_message(char *task_msg)
{
    sprintf(task_msg, "\r\nInvalid command received\r\n");
    xQueueSend(uart_q, &task_msg, portMAX_DELAY);
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
   // <-- required
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

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
  uart_send("queue\r\n");   // Send a test message over UART

    // Create a queue to store 'cmd_t' type data, with a maximum of 10 items
    cmd_q = xQueueCreate(10, sizeof(cmd_t));

    // Create another queue to store 8-byte messages, with a maximum of 10 items
    uart_q = xQueueCreate(10, 8);

    // Check if both queues were created successfully
    if((cmd_q != NULL) && (uart_q != NULL))
    {
        // Create Task 1: Menu handling task
        xTaskCreate(task1_menu, "MENU", configMINIMAL_STACK_SIZE, NULL, 1, &taskhandle1);

        // Create Task 2: Command handling task
        xTaskCreate(task2_command, "command_handle", configMINIMAL_STACK_SIZE, NULL, 2, &taskhandle2);

        // Create Task 3: Command processing task
        xTaskCreate(task3_command_process, "command_process", configMINIMAL_STACK_SIZE, NULL, 2, &taskhandle3);

        // Create Task 4: UART transmission task
        xTaskCreate(task4_uart_transmit, "uart_transmit", configMINIMAL_STACK_SIZE, NULL, 2, &taskhandle4);

        // Start UART reception in interrupt mode (receive 1 byte at a time)
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
    else
    {
        // If queue creation failed, notify via UART
        uart_send("QUEUE creation failed\r\n");
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
