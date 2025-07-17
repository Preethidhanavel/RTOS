#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart0.h"
#include "semphr.h"


void task1(void *p);
void task2(void *p);

xSemaphoreHandle xMutex=NULL;

int main()
{  
  
  UART0_CONFIG();  

  xMutex=xSemaphoreCreateMutex();  //xMutex will be storing NULL when Mutex not created
  if(xMutex!=NULL) {
    xTaskCreate(task1,"task1",128,NULL,1,NULL);
    xTaskCreate(task2,"task2",128,NULL,1,NULL);
    vTaskStartScheduler();
  }
  while(1);
}

void task1(void *p)
{
  while(1) {
    xSemaphoreTake(xMutex,portMAX_DELAY );
    uart_string("Task1 functioning\r\n");
    xSemaphoreGive( xMutex );
    vTaskDelay(50);
  }
}

void task2(void *p)
{
  while(1)
  {
    xSemaphoreTake(xMutex,portMAX_DELAY );
    uart_string("Task2 functioning\r\n");
    xSemaphoreGive(xMutex);
    vTaskDelay(50);
  }
}
