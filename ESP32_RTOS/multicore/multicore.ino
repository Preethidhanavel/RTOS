static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

static const uint32_t task_0_delay = 500;

static const int pin_1 = 13;

static SemaphoreHandle_t bin_sem;

void doTask0(void *parameters) {
  pinMode(pin_1, OUTPUT);
  while (1) {
    xSemaphoreGive(bin_sem);
    vTaskDelay(task_0_delay / portTICK_PERIOD_MS);
  }
}

void doTask1(void *parameters) {
  while (1) {
    xSemaphoreTake(bin_sem, portMAX_DELAY);
    digitalWrite(pin_1, !digitalRead(pin_1));
  }
}

void setup() {
  bin_sem = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(doTask0, "Task 0", 1024, NULL, 1, NULL, pro_cpu);
  xTaskCreatePinnedToCore(doTask1, "Task 1", 1024, NULL, 1, NULL, app_cpu);
  vTaskDelete(NULL);
}

void loop() {}
