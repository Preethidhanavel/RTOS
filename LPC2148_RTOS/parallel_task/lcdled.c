#include<FreeRTOS.H>
#include<task.h>

#define bit(x) (1<<x)

void lcd(void *);
void led(void *);

void lcd_init(void);
void cmd(unsigned char a);
void dat(unsigned char b);
void show(unsigned char *s);
void lcd_delay(void);

void lcd_init()
{
    cmd(0x38);
    cmd(0x0e);
    cmd(0x01);
    cmd(0x06);
    cmd(0x0c);
    cmd(0x80);
}

void cmd(unsigned char a)
{
    IO1CLR=0xFF070000;
    IO1SET=(a<<24);
    IO1CLR=bit(16);             
    IO1CLR=bit(17);             
    IO1SET=bit(18);             
    lcd_delay();
    IO1CLR=bit(18);             
}

void dat(unsigned char b)
{
    IO1CLR=0xFF070000;
    IO1SET=(b<<24);
    IO1SET=bit(16);             
    IO1CLR=bit(17);             
    IO1SET=bit(18);             
    lcd_delay();
    IO1CLR=bit(18);             
}

void show(unsigned char *s)
{
    while(*s) {
        dat(*s++);
    }
}

void lcd_delay()
{
    unsigned int i;
    for(i=0;i<=2000;i++);
}

int main()
{
    IO0DIR=IO1DIR=0xffffffff;
    lcd_init();
    xTaskCreate(lcd,"lcd scroll",1000,0,1,0);
    xTaskCreate(led,"led blinking",1000,0,1,0);
    vTaskStartScheduler();
}

void lcd(void *s)
{
    cmd(0x80);
    show("EMBEDDED SYSTEM");
    while(1) {
        cmd(0x18);
        vTaskDelay(1);
    }
}

void led(void *s)
{
    while(1) {
        IO0SET=0xff00;
        vTaskDelay(1);
        IO0CLR=0xff00;
        vTaskDelay(1);
    }
}