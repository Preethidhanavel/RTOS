enum {BUF_SIZE = 5};
static const int num_prod_tasks = 5;
static const int num_cons_tasks = 2;
static const int num_writes = 3;

static int buf[BUF_SIZE];
static int head = 0;
static int tail = 0;
static SemaphoreHandle_t bin_sem;
static SemaphoreHandle_t mutex;
static SemaphoreHandle_t sem_empty;
static SemaphoreHandle_t sem_filled;

void producer(void *parameters) {
  int num = *(int *)parameters;
  xSemaphoreGive(bin_sem);
  for (int i = 0; i < num_writes; i++) {
    xSemaphoreTake(sem_empty, portMAX_DELAY);
    xSemaphoreTake(mutex, portMAX_DELAY);
    buf[head] = num;
    head = (head + 1) % BUF_SIZE;
    xSemaphoreGive(mutex);
    xSemaphoreGive(sem_filled);
  }
  vTaskDelete(NULL);
}

void consumer(void *parameters) {
  int val;
  while (1) {
    xSemaphoreTake(sem_filled, portMAX_DELAY);
    xSemaphoreTake(mutex, portMAX_DELAY);
    val = buf[tail];
    tail = (tail + 1) % BUF_SIZE;
    Serial.println(val);
    xSemaphoreGive(mutex);
    xSemaphoreGive(sem_empty);
  }
}

void setup() {
  char task_name[12];
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  bin_sem = xSemaphoreCreateBinary();
  mutex = xSemaphoreCreateMutex();
  sem_empty = xSemaphoreCreateCounting(BUF_SIZE, BUF_SIZE);
  sem_filled = xSemaphoreCreateCounting(BUF_SIZE, 0);

  for (int i = 0; i < num_prod_tasks; i++) {
    sprintf(task_name, "Producer %i", i);
   xTaskCreatePinnedToCore(producer, task_name, 1024, (void *)&i, 1, NULL, 1);

    xSemaphoreTake(bin_sem, portMAX_DELAY);
  }

  for (int i = 0; i < num_cons_tasks; i++) {
    sprintf(task_name, "Consumer %i", i);
    xTaskCreatePinnedToCore(consumer, task_name, 1024,NULL, 1, NULL, 1);
  }

  xSemaphoreTake(mutex, portMAX_DELAY);
  Serial.println("All tasks created");
  xSemaphoreGive(mutex);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

