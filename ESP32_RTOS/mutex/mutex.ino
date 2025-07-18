static const int led_pin = 2;
static SemaphoreHandle_t mutex;

void blinkLED(void *parameters) 
{

  xSemaphoreTake(mutex, portMAX_DELAY);
  int num = *(int *)parameters;
  xSemaphoreGive(mutex);
  Serial.print("Received: ");
  Serial.println(num);

  while (1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(num / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(num / portTICK_PERIOD_MS);
  }
}


void setup() 
{
  long int delay_arg;
  pinMode(led_pin, OUTPUT);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Enter a number for delay (milliseconds)");
  while (Serial.available() <= 0);
    delay_arg = Serial.parseInt();
  Serial.print("Sending: ");
  Serial.println(delay_arg);
  mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(blinkLED,"Blink LED",1024,(void *)&delay_arg,1,NULL,1);
  vTaskDelay(portTICK_PERIOD_MS);
  xSemaphoreTake(mutex, portMAX_DELAY);
  Serial.println("Done!");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}