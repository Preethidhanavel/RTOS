#include <stdlib.h>

static const uint8_t buf_len = 20;  // Maximum length of serial input buffer
static const int led_pin = 2;       // LED pin
static int led_delay = 500;         // Default LED toggle delay in milliseconds

// Task to toggle LED with delay
void toggleLED(void *parameter) 
{
  while (1) 
  {
    digitalWrite(led_pin, HIGH);                     // Turn LED ON
    vTaskDelay(led_delay / portTICK_PERIOD_MS);      // Wait for led_delay
    digitalWrite(led_pin, LOW);                      // Turn LED OFF
    vTaskDelay(led_delay / portTICK_PERIOD_MS);      // Wait for led_delay
  }
}

// Task to read user input from serial and update LED delay
void readSerial(void *parameters) 
{
  char c;                     // Current character from serial
  char buf[buf_len];          // Buffer to store serial input
  uint8_t idx = 0;            // Buffer index
  memset(buf, 0, buf_len);    // Clear buffer

  while (1) 
  {
    if (Serial.available() > 0)   // Check if data is available
    {
      c = Serial.read();           // Read one character

      if (c == '\n')               // Enter key indicates end of input
      {
        led_delay = atoi(buf);     // Convert buffer string to integer
        Serial.print("Updated LED delay to: ");
        Serial.println(led_delay); // Print updated delay
        memset(buf, 0, buf_len);   // Clear buffer
        idx = 0;                   // Reset buffer index
      } 
      else 
      {
        if (idx < buf_len - 1)     // Store character in buffer if space available
        {
          buf[idx] = c;
          idx++;
        }
      }
    }
  }
}

void setup() 
{
  pinMode(led_pin, OUTPUT);           // Set LED pin as output
  Serial.begin(115200);               // Start serial communication
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for serial initialization
  Serial.println("Enter a number in milliseconds to change the LED delay.");

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(toggleLED,"Toggle LED",1024,NULL,1,NULL,1);      
  xTaskCreatePinnedToCore(readSerial,"Read Serial",1024,NULL,1,NULL,1);       

  vTaskDelete(NULL);  // Delete setup task
}

void loop() {
  // Execution should never get here, tasks handle all functionality
}
