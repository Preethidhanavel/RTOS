#include <lpc214x.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart0.h"

// Task function declarations
void task1(void *q);
void task2(void *a);

int main(void)
{
  UART0_CONFIG();  // Initialize UART0

  // Create task1 with message "HI" and priority 1
  xTaskCreate(task1, "task1", 128, "HI", 1, NULL);
  
  // Create task2 with message "HELLO" and priority 2
  xTaskCreate(task2, "task2", 128, "HELLO", 2, NULL);
  
  // Start FreeRTOS scheduler
  vTaskStartScheduler();
}

// Task1: prints its message and a status string every 1 second
void task1(void *q)
{  
  while (1) 
  { 
    uart_string((unsigned char *)q);               // Print the task message
    uart_string("---Task1 functioning\r\n");       // Print status
    vTaskDelay(1000);                              // Delay 1000 ticks
  }
}

// Task2: prints its message and a status string every 1 second
void task2(void *a)
{
  while (1) 
  {
    uart_string((unsigned char *)a);               // Print the task message
    uart_string("---Task2 functioning\r\n");       // Print status
    vTaskDelay(1000);                              // Delay 1000 ticks
  }
}
