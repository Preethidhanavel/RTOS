static const int led_pin = 2;             // GPIO pin for LED
static SemaphoreHandle_t bin_sem;         // Binary semaphore handle

// Task function to blink LED
void blinkLED(void *parameters) {
  int num = *(int *)parameters;           // Read delay value passed as parameter
  xSemaphoreGive(bin_sem);                // Signal that task has started and received the value
  Serial.print("Received: ");             
  Serial.println(num);

  pinMode(led_pin, OUTPUT);               // Set LED pin as output

  while (1) {                             // Infinite loop to blink LED
    digitalWrite(led_pin, HIGH);          // Turn LED ON
    vTaskDelay(num / portTICK_PERIOD_MS); // Delay for "num" milliseconds
    digitalWrite(led_pin, LOW);           // Turn LED OFF
    vTaskDelay(num / portTICK_PERIOD_MS); // Delay for "num" milliseconds
  }
}

void setup() {
  long int delay_arg;                      // Variable to store user input delay
  Serial.begin(115200);                    // Initialize serial communication
  vTaskDelay(1000 / portTICK_PERIOD_MS);   // Small delay to stabilize serial

  Serial.println("Enter a number for delay (milliseconds)");
  while (Serial.available() <= 0);         // Wait until user inputs a number
  delay_arg = Serial.parseInt();           // Read integer from serial

  Serial.print("Sending: ");
  Serial.println(delay_arg);

  bin_sem = xSemaphoreCreateBinary();      // Create a binary semaphore
  // Create LED blink task pinned to core 1
  xTaskCreatePinnedToCore(
    blinkLED,              // Task function
    "Blink LED",           // Task name
    1024,                  // Stack size in bytes
    (void *)&delay_arg,    // Parameter passed to task
    1,                     // Task priority
    NULL,                  // Task handle
    1                      // Core number (ESP32 has 2 cores: 0 & 1)
  );                   

  xSemaphoreTake(bin_sem, portMAX_DELAY);  // Wait until task signals semaphore
  Serial.println("Done!");                 // Confirm task started
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);   // Main loop delay (does nothing here)
}
