// Assign cores
static const BaseType_t pro_cpu = 0;   // Core 0
static const BaseType_t app_cpu = 1;   // Core 1

// Task delay in milliseconds
static const uint32_t task_0_delay = 500;

// LED pin
static const int pin_1 = 13;

// Binary semaphore handle
static SemaphoreHandle_t bin_sem;

// Task 0: Give semaphore periodically
void doTask0(void *parameters) {
  pinMode(pin_1, OUTPUT);  // Set LED pin as output
  while (1) {
    xSemaphoreGive(bin_sem);                     // Give semaphore
    vTaskDelay(task_0_delay / portTICK_PERIOD_MS); // Wait for 500 ms
  }
}

// Task 1: Wait for semaphore and toggle LED
void doTask1(void *parameters) {
  while (1) {
    xSemaphoreTake(bin_sem, portMAX_DELAY);      // Wait indefinitely for semaphore
    digitalWrite(pin_1, !digitalRead(pin_1));   // Toggle LED
  }
}

void setup() {
  // Create binary semaphore
  bin_sem = xSemaphoreCreateBinary();

  // Create Task 0 on core 0
  xTaskCreatePinnedToCore(doTask0, "Task 0", 1024, NULL, 1, NULL, pro_cpu);

  // Create Task 1 on core 1
  xTaskCreatePinnedToCore(doTask1, "Task 1", 1024, NULL, 1, NULL, app_cpu);

  vTaskDelete(NULL);  // Delete setup task
}

void loop() {
  // Execution never reaches here; tasks run on cores
}
