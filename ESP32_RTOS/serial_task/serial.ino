/*The program demonstrates FreeRTOS task management by running two tasks simultaneously, printing a message and asterisks, and showing task suspension, resumption, and deletion.*/
// Message to print
const char msg[] = "Embedded System";

// Task handles for reference (used to suspend, resume, or delete tasks)
static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;

// Task 1: Prints the message character by character every 1 second
void startTask1(void *parameter) 
{
  int msg_len = strlen(msg);

  while (1) {
    Serial.println();  // New line before printing
    for (int i = 0; i < msg_len; i++) {
      Serial.print(msg[i]);  // Print each character
    }
    Serial.println();  // New line after message
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay 1 second
  }
}

// Task 2: Continuously prints '*' every 100 ms
void startTask2(void *parameter) 
{
  while (1) {
    Serial.print('*');
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay 100 ms
  }
}
 
void setup() {

  Serial.begin(300);  // Initialize serial communication
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  Serial.println();
  Serial.println("FreeRTOS");
  Serial.print("Setup and loop task running on core ");
  Serial.print(xPortGetCoreID());  // Print current CPU core
  Serial.print(" with priority ");
  Serial.println(uxTaskPriorityGet(NULL));  // Print task priority

  // Create Task 1 with lower priority
  xTaskCreatePinnedToCore(startTask1,"Task 1",1024,NULL,1,&task_1,1);

  // Create Task 2 with higher priority
  xTaskCreatePinnedToCore(startTask2,"Task 2",1024,NULL,2,&task_2,1);
}

void loop() {

  // Suspend and resume Task 2 three times
  for (int i = 0; i < 3; i++) 
  {
    vTaskSuspend(task_2);            // Suspend Task 2
    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait 2 seconds
    vTaskResume(task_2);             // Resume Task 2
    vTaskDelay(2000 / portTICK_PERIOD_MS);  // Wait 2 seconds
  }

  // Delete Task 1 after loop
  if (task_1 != NULL) 
  {
    vTaskDelete(task_1);  // Delete Task 1
    task_1 = NULL;        // Clear the task handle
  }
}
