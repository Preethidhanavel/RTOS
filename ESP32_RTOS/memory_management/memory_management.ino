// Task to test memory allocation and stack usage
void testTask(void *parameter) 
{
  while (1)
  {
    int a = 1;
    int b[100];   // Local array on stack

    // Fill array b with values
    for (int i = 0; i < 100; i++) {
      b[i] = a + 1;
    }
    
    // Print current stack high water mark (minimum remaining stack words)
    Serial.print("High water mark (words): ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));

    // Print free heap memory before malloc
    Serial.print("Heap before malloc (bytes): ");
    Serial.println(xPortGetFreeHeapSize());

    // Allocate 1024 integers on heap
    int *ptr = (int*)pvPortMalloc(1024 * sizeof(int));
    if (ptr == NULL) {
      Serial.println("Not enough heap.");  // Allocation failed
      vPortFree(NULL);                      // Safe call if pointer is NULL
    } 
    else {   
      // Initialize allocated memory
      for (int i = 0; i < 1024; i++) {
        ptr[i] = 3;
      }
    }

    // Print free heap memory after malloc
    Serial.print("Heap after malloc (bytes): "); 
    Serial.println(xPortGetFreeHeapSize());

    // Free allocated memory (currently commented)
    // vPortFree(ptr);

    // Wait for 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);                         // Start serial
  vTaskDelay(1000 / portTICK_PERIOD_MS);        // Wait for serial init
  Serial.println();
  Serial.println("---FreeRTOS Memory---");

  // Create testTask on core 1 with 1500 bytes stack
  xTaskCreatePinnedToCore(testTask,"Test Task",1500,NULL,1,NULL,1); 

  vTaskDelete(NULL);  // Delete setup task
}

void loop() {
  // Execution should never get here
}
