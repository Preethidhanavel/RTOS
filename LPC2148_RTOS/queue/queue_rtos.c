#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart0_rtos.h"
#include "queue.h"
#include "semphr.h"

// Function declarations
void sender(void *u);
void readqueue(void *p);

xQueueHandle myqueue;  // Queue handle

int main()
{
  initserial();  // Initialize UART

  // Create a queue of length 11, each item is a char
  myqueue = xQueueCreate(11, sizeof(char));
  
  if (myqueue != NULL) {  
    // Create sender task
    xTaskCreate(sender, "sender", 128, NULL, 1, NULL);
    // Create reader task
    xTaskCreate(readqueue, "read", 128, NULL, 1, NULL);
    // Start FreeRTOS scheduler
    vTaskStartScheduler();
  }
}

// Task to send data to queue
void sender(void *u)
{
  portBASE_TYPE qstatus;
  unsigned char dat[] = "EMBEDDED";  // Data to send
  unsigned char i;

  while (1) {
    // Send each character to the queue
    for (i = 0; i < 11; i++)   
      qstatus = xQueueSendToBack(myqueue, &dat[i], 0);
    
    // If queue is full
    if (qstatus != pdPASS) {
      sendsserial("\r\n");  // Print new line
      vTaskDelay(10);       // Short delay
    }
  }
}

// Task to read data from queue
void readqueue(void *p)
{
  unsigned char receivedValue;
  portBASE_TYPE xStatus;
  
  while (1) {
    // Try to receive from queue with 100 tick timeout
    xStatus = xQueueReceive(myqueue, &receivedValue, 100);
    if (xStatus == pdPASS) {
       sendserial(receivedValue);  // Send received char via UART
     }
  }
}
