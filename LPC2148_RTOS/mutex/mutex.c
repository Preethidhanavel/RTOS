#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart0.h"
#include "semphr.h"

// Task function declarations
void task1(void *p);
void task2(void *p);

xSemaphoreHandle xMutex = NULL; // Mutex handle

int main()
{  
    UART0_CONFIG(); // Configure UART0

    // Create a mutex
    xMutex = xSemaphoreCreateMutex();  

    // Check if mutex creation successful
    if (xMutex != NULL) {
        // Create two tasks that use the mutex
        xTaskCreate(task1, "task1", 128, NULL, 1, NULL);
        xTaskCreate(task2, "task2", 128, NULL, 1, NULL);

        vTaskStartScheduler(); // Start FreeRTOS scheduler
    }

    while(1); // Should never reach here
}

// Task 1: takes mutex, prints message, releases mutex
void task1(void *p)
{
    while(1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);  // Take mutex (wait indefinitely)
        uart_string("Task1 functioning\r\n");  // Critical section
        xSemaphoreGive(xMutex);                 // Release mutex
        vTaskDelay(50);                         // Delay to allow other task to run
    }
}

// Task 2: takes mutex, prints message, releases mutex
void task2(void *p)
{
    while(1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);  // Take mutex
        uart_string("Task2 functioning\r\n");  // Critical section
        xSemaphoreGive(xMutex);                 // Release mutex
        vTaskDelay(50);                         // Delay to allow other task to run
    }
}
