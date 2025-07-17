#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "uart0.h"
#include "semphr.h"
#include "serial.h"
//#define configUSE_COUNTING_SEMAPHORES 1
void initpll(void);
void task1(void *p);
void task2(void *p);
void task3(void *p);
xSemaphoreHandle countingsemaphore;
void rx1(void)__irq
{
  static portBASE_TYPE interrupttask;
  unsigned char r;
  
  if(U0IIR&0x04) {
    r=U0RBR;
    uart_string("Received Data = ");
    U0THR=r;
    while(!(U0LSR&0x20));
    uart_string("\r\nGoing to Give Semaphore From ISR\r\n");
    xSemaphoreGiveFromISR(countingsemaphore,&interrupttask );    
  }
  VICVectAddr = 0x00;
  
}
void uart_interrupt()
{
  
  U0TER=(1<<7);
  U0IER=0x01;  //enables uart receive identification interrupt
  VICIntSelect=0x0000;     //selecting as irq
  VICIntEnable|=0x0040;   //enable uart1
  VICVectAddr2=(unsigned long int)rx1;  //assigning address
  VICVectCntl2=0x26;  //need to assign slot no of interrupt source which has been activated
}

int main()
{
  UART0_CONFIG();
	uart_interrupt();
  countingsemaphore=xSemaphoreCreateCounting(3,0);
  xTaskCreate(task1,"task1",128,NULL,1,NULL);
  xTaskCreate(task2,"task2",128,NULL,1,NULL);
	xTaskCreate(task3,"task3",128,NULL,1,NULL);
  vTaskStartScheduler();
  while(1);
  
}


void task1(void *p)
{
  while(1) 
		{
    if(countingsemaphore!=NULL) {      
      if(xSemaphoreTake(countingsemaphore,portMAX_DELAY)==pdTRUE) {
        uart_string("Task 1 Takes Semaphore\r\n");
      
      }
    }
  }
}
void task2(void *p)
{
  while(1) {
    if(countingsemaphore!=NULL) {      
      if(xSemaphoreTake(countingsemaphore,portMAX_DELAY )==pdTRUE) {
        uart_string("Task 2 Takes Semaphore");
       
      }
    }
  }
}
void task3(void *p)
{
  while(1) 
		{
    if(countingsemaphore!=NULL) {      
      if(xSemaphoreTake(countingsemaphore,portMAX_DELAY)==pdTRUE) {
        uart_string("Task 3 Takes Semaphore\r\n");
      
      }
    }
  }
}