// Queue length
static const uint8_t msg_queue_len = 5;
// Queue handle
static QueueHandle_t msg_queue;

// Task to receive and print messages from the queue
void printMessages(void *parameters) {
  int item;
  while (1) {
    // Check if an item is available in the queue
    if (xQueueReceive(msg_queue, (void *)&item, 0) == pdTRUE) {
      Serial.println(item); // Print received item
    }

    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay 500ms between checks
  }
}

void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Create a queue to hold 'msg_queue_len' integers
  msg_queue = xQueueCreate(msg_queue_len, sizeof(int));

  // Create a task to print messages from the queue
  xTaskCreatePinnedToCore(printMessages, "Print Messages", 1024, NULL, 1, NULL, 1);
}

void loop() {
  static int num = 0;

  // Send the next number to the queue
  if (xQueueSend(msg_queue, (void *)&num, 10) != pdTRUE) {
    Serial.println("Queue full"); // If queue is full, print a warning
  }
  num++; // Increment number to send next

  vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second before sending next item
}
