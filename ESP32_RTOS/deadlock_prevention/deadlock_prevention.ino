enum { NUM_TASKS = 5 };           // Total number of customer tasks
enum { TASK_STACK_SIZE = 2048 };  // Stack size for each task

// Semaphores
static SemaphoreHandle_t bin_sem;          // Binary semaphore for task creation sync
static SemaphoreHandle_t done_sem;         // Counting semaphore to track task completion
static SemaphoreHandle_t car[NUM_TASKS];   // Mutexes representing each car

// Customer task
void task(void *parameters) {
  int num;            // Customer number
  char buf[50];       // Buffer for serial messages
  int idx_1, idx_2;   // Indices of cars selected

  num = *(int *)parameters;
  xSemaphoreGive(bin_sem);  // Signal task creation to setup

  // Determine order of car picking to avoid deadlock
  if (num < (num + 1) % NUM_TASKS) {
    idx_1 = num;
    idx_2 = (num + 1) % NUM_TASKS;
  } else {
    idx_1 = (num + 1) % NUM_TASKS;
    idx_2 = num;
  }

  xSemaphoreTake(car[idx_1], portMAX_DELAY);  // Take first car
  sprintf(buf, "customer %i selected car %i", num, num);
  Serial.println(buf);

  vTaskDelay(1 / portTICK_PERIOD_MS);         // Simulate some delay

  xSemaphoreTake(car[idx_2], portMAX_DELAY);  // Take second car
  sprintf(buf, "customer %i selected car %i", num, (num + 1) % NUM_TASKS);
  Serial.println(buf);

  sprintf(buf, "customer %i is checking the quality of car", num);
  Serial.println(buf);
  vTaskDelay(10 / portTICK_PERIOD_MS);       // Simulate checking time

  xSemaphoreGive(car[idx_2]);                // Release second car
  sprintf(buf, "customer %i returned car %i", num, (num + 1) % NUM_TASKS);
  Serial.println(buf);

  xSemaphoreGive(car[idx_1]);                // Release first car
  sprintf(buf, "customer %i returned car %i", num, num);
  Serial.println(buf);

  xSemaphoreGive(done_sem);                  // Signal task completion
  vTaskDelete(NULL);                         // Delete task
}

void setup() {
  char task_name[20];

  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Create semaphores
  bin_sem = xSemaphoreCreateBinary();        // Binary semaphore for task creation sync
  done_sem = xSemaphoreCreateCounting(NUM_TASKS, 0); // Counting semaphore to track completions

  for (int i = 0; i < NUM_TASKS; i++) {
    car[i] = xSemaphoreCreateMutex();        // Each car represented as a mutex
  }

  // Create customer tasks
  for (int i = 0; i < NUM_TASKS; i++) {
    sprintf(task_name, "customer  %i", i);
    xTaskCreatePinnedToCore(task, task_name, TASK_STACK_SIZE, (void *)&i, 1, NULL, 1);
    xSemaphoreTake(bin_sem, portMAX_DELAY);  // Wait for task creation confirmation
  }

  // Wait until all tasks complete
  for (int i = 0; i < NUM_TASKS; i++) {
    xSemaphoreTake(done_sem, portMAX_DELAY);
  }

  Serial.println("Done! No deadlock occurred!"); // All tasks finished
}

void loop() {
  // Main loop does nothing; all work is in tasks
}
