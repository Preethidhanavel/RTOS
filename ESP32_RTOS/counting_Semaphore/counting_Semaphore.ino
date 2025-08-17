enum {BUF_SIZE = 5};                   // Size of shared buffer
static const int num_prod_tasks = 5;   // Number of producer tasks
static const int num_cons_tasks = 2;   // Number of consumer tasks
static const int num_writes = 3;       // Number of times each producer writes

static int buf[BUF_SIZE];              // Shared buffer
static int head = 0;                   // Index for next write
static int tail = 0;                   // Index for next read

// Semaphores
static SemaphoreHandle_t bin_sem;      // Binary semaphore for task sync
static SemaphoreHandle_t mutex;        // Mutex for buffer access
static SemaphoreHandle_t sem_empty;    // Counting semaphore for empty slots
static SemaphoreHandle_t sem_filled;   // Counting semaphore for filled slots

// Producer task
void producer(void *parameters) {
  int num = *(int *)parameters;        // Get producer number
  xSemaphoreGive(bin_sem);             // Signal task creation

  for (int i = 0; i < num_writes; i++) {
    xSemaphoreTake(sem_empty, portMAX_DELAY);  // Wait for empty slot
    xSemaphoreTake(mutex, portMAX_DELAY);      // Acquire mutex to access buffer

    buf[head] = num;                   // Write data to buffer
    head = (head + 1) % BUF_SIZE;      // Circular increment

    xSemaphoreGive(mutex);             // Release mutex
    xSemaphoreGive(sem_filled);        // Signal that slot is filled
  }
  vTaskDelete(NULL);                    // Delete task after done
}

// Consumer task
void consumer(void *parameters) {
  int val;
  while (1) {
    xSemaphoreTake(sem_filled, portMAX_DELAY); // Wait for filled slot
    xSemaphoreTake(mutex, portMAX_DELAY);      // Acquire mutex to access buffer

    val = buf[tail];                   // Read value from buffer
    tail = (tail + 1) % BUF_SIZE;      // Circular increment
    Serial.println(val);               // Print consumed value

    xSemaphoreGive(mutex);             // Release mutex
    xSemaphoreGive(sem_empty);         // Signal that slot is empty
  }
}

void setup() {
  char task_name[12];
  Serial.begin(115200);                // Initialize serial communication
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Create semaphores
  bin_sem = xSemaphoreCreateBinary();  
  mutex = xSemaphoreCreateMutex();    
  sem_empty = xSemaphoreCreateCounting(BUF_SIZE, BUF_SIZE); // Initially all slots empty
  sem_filled = xSemaphoreCreateCounting(BUF_SIZE, 0);       // Initially no filled slots

  // Create producer tasks
  for (int i = 0; i < num_prod_tasks; i++) {
    sprintf(task_name, "Producer %i", i);
    xTaskCreatePinnedToCore(producer, task_name, 1024, (void *)&i, 1, NULL, 1);
    xSemaphoreTake(bin_sem, portMAX_DELAY);  // Wait until producer task signals ready
  }

  // Create consumer tasks
  for (int i = 0; i < num_cons_tasks; i++) {
    sprintf(task_name, "Consumer %i", i);
    xTaskCreatePinnedToCore(consumer, task_name, 1024,NULL, 1, NULL, 1);
  }

  xSemaphoreTake(mutex, portMAX_DELAY);
  Serial.println("All tasks created");    // Confirm task creation
  xSemaphoreGive(mutex);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);   // Main loop delay (does nothing here)
}
