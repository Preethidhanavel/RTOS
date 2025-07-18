void testTask(void *parameter) 
{
  while (1)
   {
    int a = 1;
    int b[100];

    
    for (int i = 0; i < 100; i++) {
      b[i] = a + 1;
    }
    
    Serial.print("High water mark (words): ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
    Serial.print("Heap before malloc (bytes): ");
    Serial.println(xPortGetFreeHeapSize());
    int *ptr = (int*)pvPortMalloc(1024 * sizeof(int));
    if (ptr == NULL) {
      Serial.println("Not enough heap.");
      vPortFree(NULL);
    } 
    else {   
      for (int i = 0; i < 1024; i++) {
        ptr[i] = 3;
      }
    }

    Serial.print("Heap after malloc (bytes): "); 
    Serial.println(xPortGetFreeHeapSize());

    // Free up our allocated memory
    //vPortFree(ptr);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {


  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Memory---");
  xTaskCreatePinnedToCore(testTask,"Test Task",1500,NULL,1,NULL,1); 
  vTaskDelete(NULL);
}

void loop() {
  // Execution should never get here
}
