enum { NUM_TASKS = 5 };
enum { TASK_STACK_SIZE = 2048 };

static SemaphoreHandle_t bin_sem;
static SemaphoreHandle_t done_sem;
static SemaphoreHandle_t car[NUM_TASKS];

void task(void *parameters) {
  int num;
  char buf[50];
  int idx_1;
  int idx_2;

  num = *(int *)parameters;
  xSemaphoreGive(bin_sem);

  if (num < (num + 1) % NUM_TASKS) {
    idx_1 = num;
    idx_2 = (num + 1) % NUM_TASKS;
  } else {
    idx_1 = (num + 1) % NUM_TASKS;
    idx_2 = num;
  }

  xSemaphoreTake(car[idx_1], portMAX_DELAY);
  sprintf(buf, "customer %i selected car %i", num, num);
  Serial.println(buf);

  vTaskDelay(1 / portTICK_PERIOD_MS);

  xSemaphoreTake(car[idx_2], portMAX_DELAY);
  sprintf(buf, "customer %i selected car %i", num, (num + 1) % NUM_TASKS);
  Serial.println(buf);

  sprintf(buf, "customer %i is checking the quality of car", num);
  Serial.println(buf);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  xSemaphoreGive(car[idx_2]);
  sprintf(buf, "customer %i returned car %i", num, (num + 1) % NUM_TASKS);
  Serial.println(buf);

  xSemaphoreGive(car[idx_1]);
  sprintf(buf, "customer %i returned car %i", num, num);
  Serial.println(buf);

  xSemaphoreGive(done_sem);
  vTaskDelete(NULL);
}

void setup() {
  char task_name[20];

  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);


  bin_sem = xSemaphoreCreateBinary();
  done_sem = xSemaphoreCreateCounting(NUM_TASKS, 0);
  for (int i = 0; i < NUM_TASKS; i++) {
    car[i] = xSemaphoreCreateMutex();
  }

  for (int i = 0; i < NUM_TASKS; i++) {
    sprintf(task_name, "customer  %i", i);
    xTaskCreatePinnedToCore(task, task_name, TASK_STACK_SIZE, (void *)&i, 1, NULL, 1);
    xSemaphoreTake(bin_sem, portMAX_DELAY);
  }

  for (int i = 0; i < NUM_TASKS; i++) {
    xSemaphoreTake(done_sem, portMAX_DELAY);
  }

  Serial.println("Done! No deadlock occurred!");
}

void loop() {}
