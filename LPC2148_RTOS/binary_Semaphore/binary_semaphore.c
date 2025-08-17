#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart0.h"
#include "semphr.h"

// Task function declarations
void task1(void *q);
void task2(void *a);

xSemaphoreHandle binarysem; // Binary semaphore handle

int main(void)
{
    UART0_CONFIG(); // Initialize UART0 for printing messages

    // Create a binary semaphore
    binarysem = xSemaphoreCreateBinary();

    // Create two tasks with same priority
    xTaskCreate(task1, "task1", 128, NULL, 1, NULL);
    xTaskCreate(task2, "task2", 128, NULL, 1, NULL);

    xSemaphoreGive(binarysem); // Give initial semaphore so one task can start

    vTaskStartScheduler(); // Start FreeRTOS scheduler
    while(1); // Should never reach here
}

// Task 1 function
void task1(void *q)
{  
    while(1) 
    {  
        xSemaphoreTake(binarysem, portMAX_DELAY);  // Wait for semaphore
        uart_string("Task1 functioning\r\n");      // Print message via UART
        xSemaphoreGive(binarysem);                 // Release semaphore
        vTaskDelay(100);                           // Delay for 100 ticks
    }
}

// Task 2 function
void task2(void *a)
{
    while(1) 
    {
        xSemaphoreTake(binarysem, portMAX_DELAY);  // Wait for semaphore
        uart_string("Task2 functioning\r\n");      // Print message via UART
        xSemaphoreGive(binarysem);                 // Release semaphore
        vTaskDelay(100);                           // Delay for 100 ticks
    }
}
