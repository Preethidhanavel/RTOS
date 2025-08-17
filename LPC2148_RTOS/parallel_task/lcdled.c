#include <FreeRTOS.H>
#include <task.h>

#define bit(x) (1<<x)  // Macro to set a single bit

// Function declarations
void lcd(void *);
void led(void *);

void lcd_init(void);
void cmd(unsigned char a);
void dat(unsigned char b);
void show(unsigned char *s);
void lcd_delay(void);

// LCD Initialization
void lcd_init()
{
    cmd(0x38);  // Function set: 8-bit, 2 lines, 5x7 font
    cmd(0x0e);  // Display ON, cursor ON
    cmd(0x01);  // Clear display
    cmd(0x06);  // Entry mode: increment cursor
    cmd(0x0c);  // Display ON, cursor OFF
    cmd(0x80);  // Set cursor to first line, first position
}

// Send command to LCD
void cmd(unsigned char a)
{
    IO1CLR = 0xFF070000;     // Clear data lines
    IO1SET = (a << 24);      // Put command on data lines
    IO1CLR = bit(16);        // RS = 0 (command)
    IO1CLR = bit(17);        // RW = 0 (write)
    IO1SET = bit(18);        // EN = 1 (enable)
    lcd_delay();             // Small delay
    IO1CLR = bit(18);        // EN = 0
}

// Send data to LCD
void dat(unsigned char b)
{
    IO1CLR = 0xFF070000;     // Clear data lines
    IO1SET = (b << 24);      // Put data on data lines
    IO1SET = bit(16);        // RS = 1 (data)
    IO1CLR = bit(17);        // RW = 0 (write)
    IO1SET = bit(18);        // EN = 1
    lcd_delay();             // Small delay
    IO1CLR = bit(18);        // EN = 0
}

// Display string on LCD
void show(unsigned char *s)
{
    while (*s) {
        dat(*s++);           // Send each character
    }
}

// Simple delay for LCD timing
void lcd_delay()
{
    unsigned int i;
    for (i = 0; i <= 2000; i++);
}

int main()
{
    IO0DIR = IO1DIR = 0xffffffff;   // Set all pins as output
    lcd_init();                      // Initialize LCD

    // Create FreeRTOS tasks
    xTaskCreate(lcd, "lcd scroll", 1000, 0, 1, 0);
    xTaskCreate(led, "led blinking", 1000, 0, 1, 0);

    vTaskStartScheduler();           // Start the FreeRTOS scheduler
}

// LCD task: scrolls the message
void lcd(void *s)
{
    cmd(0x80);                       // Set cursor to first line
    show("EMBEDDED SYSTEM");          // Display message
    while (1) {
        cmd(0x18);                    // Shift display left
        vTaskDelay(1);                // Delay for scrolling
    }
}

// LED task: simple blinking
void led(void *s)
{
    while (1) {
        IO0SET = 0xff00;              // Turn ON LEDs
        vTaskDelay(1);                // Delay
        IO0CLR = 0xff00;              // Turn OFF LEDs
        vTaskDelay(1);                // Delay
    }
}
