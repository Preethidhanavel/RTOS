// Time constants for task execution (milliseconds)
TickType_t cs_wait = 250;    // Critical section duration
TickType_t med_wait = 5000;  // Medium task work duration

// Mutex handle
SemaphoreHandle_t lock;

// Low-priority task
void doTaskL(void *parameters) {
  TickType_t timestamp;
  while (1) {
    Serial.println("Task L trying to take lock...");
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;   // Record start time
    xSemaphoreTake(lock, portMAX_DELAY);                    // Take mutex (block if not available)
    Serial.print("Task L got lock. Spent ");
    Serial.print((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp); // Time waited
    Serial.println(" ms waiting for lock. Doing some work...");

    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < cs_wait); // Simulate work

    Serial.println("Task L releasing lock.");
    xSemaphoreGive(lock);                                    // Release mutex
    vTaskDelay(500 / portTICK_PERIOD_MS);                    // Delay before next iteration
  }
}

// Medium-priority task
void doTaskM(void *parameters) {
  TickType_t timestamp;
  while (1) {
    Serial.println("Task M doing some work...");
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < med_wait); // Simulate long work
    Serial.println("Task M done!");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// High-priority task
void doTaskH(void *parameters) {
  TickType_t timestamp;
  while (1) {
    Serial.println("Task H trying to take lock...");
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;   // Record start time
    xSemaphoreTake(lock, portMAX_DELAY);                    // Take mutex
    Serial.print("Task H got lock. Spent ");
    Serial.print((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp);
    Serial.println(" ms waiting for lock. Doing some work...");

    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < cs_wait); // Simulate work

    Serial.println("Task H releasing lock.");
    xSemaphoreGive(lock);                                    // Release mutex
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("---FreeRTOS Mutex Lock Demo---");

  lock = xSemaphoreCreateMutex();                             // Create a mutex

  // Create tasks with different priorities
  xTaskCreatePinnedToCore(doTaskL, "Task L", 1024, NULL, 1, NULL, 1); // Low priority
  vTaskDelay(1 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(doTaskH, "Task H", 1024, NULL, 3, NULL, 1); // High priority
  xTaskCreatePinnedToCore(doTaskM, "Task M", 1024, NULL, 2, NULL, 1); // Medium priority

  vTaskDelete(NULL);                                           // Delete setup task
}

void loop() {
  // Nothing needed here; tasks handle execution
}
