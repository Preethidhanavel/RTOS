// Function declarations
void UART0_CONFIG(void);            // Configure UART0
void UART0_TX(unsigned char);       // Transmit a single byte
unsigned char UART0_RX(void);       // Receive a single byte
void uart_string(unsigned char *);  // Transmit a string
void uart_interrupt(void);          // UART interrupt (prototype)

// UART0 initialization
void UART0_CONFIG(void)
{
    PINSEL0 |= 0x05;    // Select pins for TXD0 and RXD0
    U0LCR = 0x83;       // 8-bit data, 1 stop bit, DLAB = 1
    U0DLL = 97;         // Set baud rate low byte (for 9600)
    U0DLM = 0;          // Set baud rate high byte
    U0LCR = 0x03;       // 8-bit data, 1 stop bit, DLAB = 0
}

// Transmit a single byte over UART0
void UART0_TX(unsigned char d)
{
    U0THR = d;                  // Load data into transmit holding register
    while (((U0LSR >> 5) & 1) == 0);  // Wait until THR is empty
}

// Receive a single byte from UART0
unsigned char UART0_RX(void)
{
    while ((U0LSR & 1) == 0);   // Wait until data is ready
    return U0RBR;               // Read received byte
}

// Send a string over UART0
void uart_string(unsigned char *s)
{
    while (*s) {
        UART0_TX(*s++);          // Transmit each character
        // delay_ms(150);        // Optional delay if needed
    }
}
