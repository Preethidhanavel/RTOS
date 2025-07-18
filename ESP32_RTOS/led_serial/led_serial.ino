#include <stdlib.h>
static const uint8_t buf_len = 20;
static const int led_pin = 2;
static int led_delay = 500;   

void toggleLED(void *parameter) 
{
  while (1) 
  {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(led_delay / portTICK_PERIOD_MS);
  }
}

void readSerial(void *parameters) 
{

  char c;
  char buf[buf_len];
  uint8_t idx = 0;
  memset(buf, 0, buf_len);

  while (1) 
  {

    if (Serial.available() > 0) 
    {
      c = Serial.read();

      if (c == '\n') 
      {
        led_delay = atoi(buf);
        Serial.print("Updated LED delay to: ");
        Serial.println(led_delay);
        memset(buf, 0, buf_len);
        idx = 0;
      } 
      else 
      {
        if (idx < buf_len - 1) 
        {
          buf[idx] = c;
          idx++ ;
        }
      }
    }
  }
}


void setup() 
{

  pinMode(led_pin, OUTPUT);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Enter a number in milliseconds to change the LED delay.");
  xTaskCreatePinnedToCore(toggleLED,"Toggle LED",1024,NULL,1,NULL,1);      
  xTaskCreatePinnedToCore(readSerial,"Read Serial",1024,NULL,1,NULL,1);       
  vTaskDelete(NULL);
}

void loop() {
  // Execution should never get here
}