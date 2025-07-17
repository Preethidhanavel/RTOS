#include <lpc214x.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart0.h"

void task1(void *q);
void task2(void *a);


int main(void)
{
 
  UART0_CONFIG();
	xTaskCreate(task1,"task1",128,"HI",1,NULL);
  xTaskCreate(task2,"task2",128,"HELLO",2,NULL);
  vTaskStartScheduler();
}
void task1(void *q)
{  
  while(1) 
	{ 
		
		uart_string((unsigned char *)q);
    uart_string("---Task1 functioning\r\n");
    vTaskDelay(1000);
  }
}

void task2(void *a)
{
  while(1) 
	{
		uart_string((unsigned char *)a);
    uart_string("---Task2 functioning\r\n");
    vTaskDelay(1000);
  }
}

