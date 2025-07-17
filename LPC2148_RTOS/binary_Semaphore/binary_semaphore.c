#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart0.h"
#include "semphr.h"

void task1(void *q);
void task2(void *a);


xSemaphoreHandle binarysem;

int main(void)
{
  
  UART0_CONFIG();
  binarysem=xSemaphoreCreateBinary();
  xTaskCreate(task1,"task1",128,NULL,1,NULL);
  xTaskCreate(task2,"task2",128,NULL,1,NULL);
	xSemaphoreGive(binarysem);
  vTaskStartScheduler();
  while(1);
}
void task1(void *q)
{  
  while(1) 
	{  
    xSemaphoreTake(binarysem,portMAX_DELAY);
    uart_string("Task1 functioning\r\n");
    xSemaphoreGive(binarysem);
    vTaskDelay(100);
  }
}

void task2(void *a)
{
  while(1) 
	{
    xSemaphoreTake(binarysem,portMAX_DELAY);
    uart_string("Task2 functioning\r\n");
    xSemaphoreGive(binarysem);
    vTaskDelay(100);
  }
}
