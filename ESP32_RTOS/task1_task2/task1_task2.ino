
#define LED_ONE 2
#define LED_TWO 15

TaskHandle_t task1Handle = NULL; 

void task1(void *parameters) {
    Serial.println("Task 1 started");
    int *args = (int *)parameters; 
    int delayOn = args[0];         
    int delayOff = args[1];       

    while (true) {
        digitalWrite(LED_ONE, HIGH);
        vTaskDelay(delayOn / portTICK_PERIOD_MS); 
        digitalWrite(LED_ONE, LOW);
        vTaskDelay(delayOff / portTICK_PERIOD_MS); 
    }
}
void task2(void *parameters) {
    Serial.println("Task 2 started.");
    while (true) {
        digitalWrite(LED_TWO, HIGH);
        vTaskDelay(500 / portTICK_PERIOD_MS); 
        digitalWrite(LED_TWO, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS); 
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Initializing tasks...");

    pinMode(LED_ONE, OUTPUT);
    pinMode(LED_TWO, OUTPUT);

    static int argArray[2] = {1000, 250}; 

    xTaskCreate(task1, "Task 1", 1000, (void *)argArray, 1, &task1Handle);
    xTaskCreate(task2, "Task 2", 1000, NULL, 1, NULL);

    Serial.println("Tasks created.");
}

void loop() {
    vTaskDelay(5000 / portTICK_PERIOD_MS); 

    Serial.println("Loop: Suspending Task 1...");
    vTaskSuspend(task1Handle);
    vTaskDelay(5000 / portTICK_PERIOD_MS); 

    Serial.println("Loop: Resuming Task 1...");
    vTaskResume(task1Handle); 

    vTaskSuspend(NULL); 
}
