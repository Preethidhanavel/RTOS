static const TickType_t dim_delay = 5000 / portTICK_PERIOD_MS; // LED auto-off delay (5 seconds)
static const int led_pin = 2;                                  // LED connected to pin 2
static TimerHandle_t one_shot_timer = NULL;                    // One-shot timer handle

// Timer callback to turn off the LED
void autoDimmerCallback(TimerHandle_t xTimer) {
  digitalWrite(led_pin, LOW); // Turn off LED
}

// Task to read CLI input and control LED
void doCLI(void *parameters) {
  char c;
  pinMode(led_pin, OUTPUT); // Set LED pin as output
  while (1) {
    if (Serial.available() > 0) {      // If serial data is available
      c = Serial.read();               // Read a character
      Serial.print(c);                 // Echo character
      digitalWrite(led_pin, HIGH);     // Turn on LED
      xTimerStart(one_shot_timer, portMAX_DELAY); // Start timer to auto-turn-off LED
    }
  }
}

void setup() {
  Serial.begin(115200);               // Initialize serial communication
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
  // Create a one-shot timer for auto-dimming the LED
  one_shot_timer = xTimerCreate("One-shot timer", dim_delay, pdFALSE, (void *)0, autoDimmerCallback);

  // Create CLI task
  xTaskCreatePinnedToCore(doCLI, "Do CLI", 1024, NULL, 1, NULL, 1);

  vTaskDelete(NULL); // Delete setup task
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Nothing in loop, task handles everything
}
