// LED pin
static const int led_pin = 2;

// Mutex handle
static SemaphoreHandle_t mutex;

// Task to blink LED
void blinkLED(void *parameters) 
{
  xSemaphoreTake(mutex, portMAX_DELAY);            // Take mutex to safely read shared data
  int num = *(int *)parameters;                    // Read delay value passed from setup
  xSemaphoreGive(mutex);                            // Release mutex

  Serial.print("Received: ");
  Serial.println(num);

  while (1) {
    digitalWrite(led_pin, HIGH);                   // Turn LED ON
    vTaskDelay(num / portTICK_PERIOD_MS);          // Wait for 'num' milliseconds
    digitalWrite(led_pin, LOW);                    // Turn LED OFF
    vTaskDelay(num / portTICK_PERIOD_MS);          // Wait for 'num' milliseconds
  }
}

void setup() 
{
  long int delay_arg;

  pinMode(led_pin, OUTPUT);                        // Set LED pin as output
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);          // Small delay for Serial to initialize

  Serial.println("Enter a number for delay (milliseconds)");
  while (Serial.available() <= 0);                // Wait for user input
  delay_arg = Serial.parseInt();                  // Read input number
  Serial.print("Sending: ");
  Serial.println(delay_arg);

  mutex = xSemaphoreCreateMutex();                // Create a mutex
  xTaskCreatePinnedToCore(blinkLED, "Blink LED", 1024, (void *)&delay_arg, 1, NULL, 1); // Create LED task

  vTaskDelay(portTICK_PERIOD_MS);                 // Small delay
  xSemaphoreTake(mutex, portMAX_DELAY);           // Ensure main waits until task has read value
  Serial.println("Done!");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);          // Nothing in loop; tasks run independently
}
