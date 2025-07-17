void UART0_CONFIG(void);
void UART0_TX(unsigned char);
unsigned char UART0_RX(void);
void string(unsigned char *);
void uart_interrupt(void);

void UART0_CONFIG(void)
{
 PINSEL0 |=0X05;
 U0LCR =0X83;
 U0DLL=97;
 U0DLM=0;
 U0LCR=0X03;

}
void UART0_TX(unsigned char d)
{
 U0THR=d;
 while(((U0LSR>>5)&1)==0);
}
unsigned char UART0_RX(void)
{
 
 while((U0LSR&1)==0);
 return U0RBR;
}


void uart_string(unsigned char *s)
{
 while(*s)
 {
  	UART0_TX(*s++);
	 //delay_ms(150);
 }
}
