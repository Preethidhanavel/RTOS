#define LED_ONE 2
#define LED_TWO 15

TaskHandle_t task1Handle = NULL; 

// Task 1: Blink LED_ONE with variable ON/OFF delay
void task1(void *parameters) {
    Serial.println("Task 1 started");
    int *args = (int *)parameters; 
    int delayOn = args[0];       // ON duration
    int delayOff = args[1];      // OFF duration

    while (true) {
        digitalWrite(LED_ONE, HIGH);         // Turn LED_ONE ON
        vTaskDelay(delayOn / portTICK_PERIOD_MS);  
        digitalWrite(LED_ONE, LOW);          // Turn LED_ONE OFF
        vTaskDelay(delayOff / portTICK_PERIOD_MS); 
    }
}

// Task 2: Blink LED_TWO with fixed 500ms ON/OFF
void task2(void *parameters) {
    Serial.println("Task 2 started.");
    while (true) {
        digitalWrite(LED_TWO, HIGH);         // Turn LED_TWO ON
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(LED_TWO, LOW);          // Turn LED_TWO OFF
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Initializing tasks...");

    pinMode(LED_ONE, OUTPUT);  // Configure LED_ONE as output
    pinMode(LED_TWO, OUTPUT);  // Configure LED_TWO as output

    static int argArray[2] = {1000, 250}; // Task1 ON=1000ms, OFF=250ms

    // Create Task 1
    xTaskCreate(task1, "Task 1", 1000, (void *)argArray, 1, &task1Handle);
    // Create Task 2
    xTaskCreate(task2, "Task 2", 1000, NULL, 1, NULL);

    Serial.println("Tasks created.");
}

void loop() {
    vTaskDelay(5000 / portTICK_PERIOD_MS); 

    Serial.println("Loop: Suspending Task 1...");
    vTaskSuspend(task1Handle);          // Suspend Task 1
    vTaskDelay(5000 / portTICK_PERIOD_MS); 

    Serial.println("Loop: Resuming Task 1...");
    vTaskResume(task1Handle);           // Resume Task 1

    vTaskSuspend(NULL);                 // Suspend loop task indefinitely
}
