const char msg[] = "Embedded System";

static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;


void startTask1(void *parameter) 
{
  int msg_len = strlen(msg);

  while (1) {
    Serial.println();
    for (int i = 0; i < msg_len; i++) {
      Serial.print(msg[i]);
    }
    Serial.println();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void startTask2(void *parameter) 
{
  while (1) {
    Serial.print('*');
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
 
void setup() {

  Serial.begin(300);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("FreeRTOS");
  Serial.print("Setup and loop task running on core ");
  Serial.print(xPortGetCoreID());
  Serial.print(" with priority ");
  Serial.println(uxTaskPriorityGet(NULL));

  xTaskCreatePinnedToCore(startTask1,"Task 1",1024,NULL,1,&task_1,1);
  xTaskCreatePinnedToCore(startTask2,"Task 2",1024,NULL,2,&task_2,1);

}

void loop() {

  
  for (int i = 0; i < 3; i++) 
  {
    vTaskSuspend(task_2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskResume(task_2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  if (task_1 != NULL) 
  {
    vTaskDelete(task_1);
    task_1 = NULL;
  }
}
