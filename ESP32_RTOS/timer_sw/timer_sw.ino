static TimerHandle_t one_shot_timer = NULL;     // Handle for one-shot timer
static TimerHandle_t auto_reload_timer = NULL;  // Handle for auto-reload timer

// Timer callback function
void myTimerCallback(TimerHandle_t xTimer) {
  // Check which timer triggered using Timer ID
  if ((uint32_t)pvTimerGetTimerID(xTimer) == 0) {
    Serial.println("One-shot timer expired");   // Message for one-shot timer
  }
  if ((uint32_t)pvTimerGetTimerID(xTimer) == 1) {
    Serial.println("Auto-reload timer expired"); // Message for auto-reload timer
  }
}

void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);       // Small delay for serial start

  // Create a one-shot timer: expires once after 2000ms
  one_shot_timer = xTimerCreate("One-shot timer", 2000 / portTICK_PERIOD_MS, pdFALSE, (void *)0, myTimerCallback);
  
  // Create an auto-reload timer: triggers every 1000ms
  auto_reload_timer = xTimerCreate("Auto-reload timer", 1000 / portTICK_PERIOD_MS, pdTRUE, (void *)1, myTimerCallback);

  // Check if timers were created successfully
  if (one_shot_timer == NULL || auto_reload_timer == NULL) {
    Serial.println("Could not create one of the timers");
  } else {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Starting timers...");
    xTimerStart(one_shot_timer, portMAX_DELAY);    // Start one-shot timer
    xTimerStart(auto_reload_timer, portMAX_DELAY); // Start auto-reload timer
  }

  vTaskDelete(NULL);  // Delete setup task
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);  // Nothing to do here, loop just waits
}
