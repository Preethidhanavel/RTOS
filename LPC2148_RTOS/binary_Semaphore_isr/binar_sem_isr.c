#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart0.h"

// Function declarations
void interrupt_config(void);
void recsemaphore(void *q);

xSemaphoreHandle binarysemaphore; // Binary semaphore handle

// UART receive interrupt service routine
void uart_recieve(void) __irq
{
    unsigned char r;
    static portBASE_TYPE interrupttask;
    interrupttask = pdFALSE;

    // Check if Receive Data Available interrupt occurred
    if (U0IIR & 0x04) {
        r = U0RBR; // Read received character
        uart_string("Received Data : ");
        
        U0THR = r;  // Echo back the received character
        while (!(U0LSR & 0x20)); // Wait until transmit buffer is empty
        
        uart_string("\r\n");
        uart_string("Going to give semaphore\r\n");
        
        // Give semaphore from ISR to unblock a task
        xSemaphoreGiveFromISR(binarysemaphore, &interrupttask);
        
        uart_string("Semaphore has given\r\n");
    }

    VICVectAddr = 0x00; // Clear interrupt in VIC
}

int main()
{
    vSemaphoreCreateBinary(binarysemaphore); // Create a binary semaphore
    UART0_CONFIG();                           // Configure UART0
    interrupt_config();                        // Configure UART interrupt

    // Take semaphore initially so ISR can give it
    xSemaphoreTake(binarysemaphore, portMAX_DELAY);

    // Create a task to handle semaphore reception
    xTaskCreate(recsemaphore, "intertask", 128, NULL, 1, NULL);

    vTaskStartScheduler(); // Start FreeRTOS scheduler

    while(1); // Should never reach here
}

// Task to receive semaphore from ISR
void recsemaphore(void *q)
{
    while (1) 
    {
        if (xSemaphoreTake(binarysemaphore, portMAX_DELAY) == pdTRUE) {
            uart_string("Received Semaphore From ISR\r\n"); // Print when semaphore is received
        }
    }
}

// Configure UART interrupt
void interrupt_config()
{
    U0TER = (1 << 7);      // Enable transmitter
    U0IER = 0x01;          // Enable UART Receive Data Available interrupt
    VICIntSelect = 0x0000; // Select as IRQ
    VICIntEnable |= 0x0040; // Enable UART0 interrupt in VIC
    VICVectAddr2 = (unsigned long int)uart_recieve; // Set ISR address
    VICVectCntl2 = 0x26;   // Set slot number and enable it
}
