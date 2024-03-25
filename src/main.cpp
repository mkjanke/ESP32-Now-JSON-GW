#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>

#include "settings.h"

TaskHandle_t xhandleSerialWriteHandle = NULL;
TaskHandle_t xhandleSerialReadHandle = NULL;
static QueueHandle_t send_to_Serial_queue;

// MAC Address of the receiver
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

// Background Task
// Camp out on Serial port, wait for '\n'.
// Send message to ESP-NOW
void task_read_Serial_Write_ESP_NOW(void *parameter) {
  Serial.setTimeout(20); // 20ms
  for (;;) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // len = 0;
    if (Serial.available()) {
    // if (Serial.available() > 6) {
      char sendBuffer[ESP_BUFFER_SIZE] = {0};
      int len = 0;
      len = Serial.readBytesUntil('\n', sendBuffer, ESP_BUFFER_SIZE);
      // Serial.println(sendBuffer);
      if (len > 6) {
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)sendBuffer, len);
        if (result != ESP_OK) {
            Serial.print("ESP-NOW Send Error: ");
            Serial.println(esp_err_to_name(result));
        }
      } else {
        Serial.println("Short Serial Read");
      }
    }
  }
  Serial.println("task_read_Serial_Write_ESP_NOW crashed");
}

// Background Task
// Dequeue JSON message from send queue and output to Serial
void task_read_ESP_NOW_Write_Serial(void *parameter) {
  char receiveBuffer[ESP_BUFFER_SIZE] = {0};
  for (;;) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Dequeue
    if (xQueueReceive(send_to_Serial_queue, receiveBuffer, portMAX_DELAY) == pdTRUE) {
      // Write to Serial
      Serial.print("JSON>> ");
      Serial.println(receiveBuffer);
      Serial.flush();                   // Wait for TX to complete
      for (size_t _i = 0; _i < ESP_BUFFER_SIZE; _i++) {
          receiveBuffer[_i] = 0;
      }
    }
  }
}

// ESP-Now message sent callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) Serial.println("Delivery Fail");
}

// ESP-NOW message recieved callback
// Queue ESP-NOW message to Serial handling queue
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (xQueueSend(send_to_Serial_queue, (void *)incomingData, 0) != pdTRUE) {
    Serial.println("Error sending to queue");
  }
}

// Initialize ESP_NOW interface. Call once from setup()
bool initEspNow() {
  // Set WIFI mode to STA mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  } else {
    Serial.println("esp_now initialized");
  }
  // Register Callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.printf("%s Is Now Woke!\n", DEVICE_NAME);

  // Start esp-now
  if (!initEspNow()) {
    return;
  };

  // Set up queues and tasks
  send_to_Serial_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, ESP_BUFFER_SIZE);
  if (send_to_Serial_queue == NULL) {
    Serial.println("Create Queue failed");
    return;
  } 
  xTaskCreate(task_read_ESP_NOW_Write_Serial, "Serial Write Handler", 2048, NULL, 4, &xhandleSerialWriteHandle);
  xTaskCreate(task_read_Serial_Write_ESP_NOW, "Serial Read Handler", 2048, NULL, 4, &xhandleSerialReadHandle);
}

void loop() {
  // Send heartbeat JSON doc back to Pi via serial port
  uint64_t now = esp_timer_get_time() / 1000 / 1000;
  String jsonsend = "";
  StaticJsonDocument<200> doc;
  doc.clear();
  doc["D"] = DEVICE_NAME;  //
  doc["T"] = long(now);
  doc["S"] = uxTaskGetStackHighWaterMark(xhandleSerialWriteHandle);
  doc["R"] = uxTaskGetStackHighWaterMark(xhandleSerialReadHandle);
  doc["H"] = esp_get_minimum_free_heap_size();
  doc["Q"] = uxQueueMessagesWaiting(send_to_Serial_queue);
  serializeJson(doc, jsonsend);  // Serilize JSON
  if (xQueueSend(send_to_Serial_queue, jsonsend.c_str(), 0) != pdTRUE) {
    Serial.println("Error sending to queue");
  }
  vTaskDelay(HEARTBEAT / portTICK_PERIOD_MS);
}
