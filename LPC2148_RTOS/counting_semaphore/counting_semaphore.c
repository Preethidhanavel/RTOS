#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart0.h"
#include "semphr.h"
#include "serial.h"

// Function declarations
void initpll(void);
void task1(void *p);
void task2(void *p);
void task3(void *p);

xSemaphoreHandle countingsemaphore; // Counting semaphore handle

// UART receive ISR
void rx1(void) __irq
{
    static portBASE_TYPE interrupttask;
    unsigned char r;

    // Check if Receive Data Available interrupt occurred
    if (U0IIR & 0x04) {
        r = U0RBR; // Read received character
        uart_string("Received Data = ");
        
        U0THR = r; // Echo the received character
        while (!(U0LSR & 0x20)); // Wait until transmit buffer empty
        
        uart_string("\r\nGoing to Give Semaphore From ISR\r\n");
        
        // Give counting semaphore from ISR
        xSemaphoreGiveFromISR(countingsemaphore, &interrupttask);
    }
    
    VICVectAddr = 0x00; // Clear interrupt in VIC
}

// UART interrupt configuration
void uart_interrupt()
{
    U0TER = (1 << 7);      // Enable UART transmitter
    U0IER = 0x01;          // Enable UART Receive Data Available interrupt
    VICIntSelect = 0x0000; // Select as IRQ
    VICIntEnable |= 0x0040; // Enable UART0 interrupt in VIC
    VICVectAddr2 = (unsigned long int)rx1; // Set ISR address
    VICVectCntl2 = 0x26;   // Set slot number and enable
}

int main()
{
    UART0_CONFIG();        // Configure UART0
    uart_interrupt();      // Configure UART interrupt
    
    // Create a counting semaphore with max count 3, initial count 0
    countingsemaphore = xSemaphoreCreateCounting(3, 0);

    // Create three tasks
    xTaskCreate(task1, "task1", 128, NULL, 1, NULL);
    xTaskCreate(task2, "task2", 128, NULL, 1, NULL);
    xTaskCreate(task3, "task3", 128, NULL, 1, NULL);

    vTaskStartScheduler(); // Start FreeRTOS scheduler
    while (1);             // Should never reach here
}

// Task 1: waits for counting semaphore from ISR
void task1(void *p)
{
    while (1) 
    {
        if (countingsemaphore != NULL) {
            if (xSemaphoreTake(countingsemaphore, portMAX_DELAY) == pdTRUE) {
                uart_string("Task 1 Takes Semaphore\r\n");
            }
        }
    }
}

// Task 2: waits for counting semaphore from ISR
void task2(void *p)
{
    while (1) 
    {
        if (countingsemaphore != NULL) {
            if (xSemaphoreTake(countingsemaphore, portMAX_DELAY) == pdTRUE) {
                uart_string("Task 2 Takes Semaphore\r\n");
            }
        }
    }
}

// Task 3: waits for counting semaphore from ISR
void task3(void *p)
{
    while (1) 
    {
        if (countingsemaphore != NULL) {
            if (xSemaphoreTake(countingsemaphore, portMAX_DELAY) == pdTRUE) {
                uart_string("Task 3 Takes Semaphore\r\n");
            }
        }
    }
}
