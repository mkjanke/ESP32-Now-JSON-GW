/**
 * ESP_NOW <-> Serial gateway
 *
 * Receives JSON formatted packets via ESP-NOW, forwards serialized JSON documents to serial port.
 * Listens on Serial port for serialized JSON documents, forwards serialized JSON documents to
 * ESP-NOW broadcast address
 *
 * Companion program to other ESP-NOW applications in this repository
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>

#include "settings.h"

/// @brief handle to task that reads msgs from ESP-NOW and
///        writes msgs to Serial port
TaskHandle_t xhandleSerialWriteHandle = NULL;

/// @brief handle to task that reads msgs from Serial port
///        and forwards to ESP-NOW
TaskHandle_t xhandleSerialReadHandle = NULL;

/// @brief Queue to handle messages received from ESP-NOW
///        and sent to Serial port
static QueueHandle_t send_to_Serial_queue;

/// @brief MAC Address of the receiver
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

esp_now_peer_info_t peerInfo;


/// @brief Background Task to listen on Serial port, wait for '\n'.
///        then broadcast messages via ESP-NOW
/// @param parameter
void task_read_Serial_Write_ESP_NOW(void *parameter) {
  Serial.setTimeout(20);  // 20ms
  for (;;) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if (Serial.available()) {
      char sendBuffer[ESP_BUFFER_SIZE] = {0};
      int len = 0;
      len = Serial.readBytesUntil('\n', sendBuffer, ESP_BUFFER_SIZE);
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

/// @brief Background Task to dequeue JSON message from send_to_Serial_queue
///        and output msg to Serial port
/// @param parameter
void task_read_ESP_NOW_Write_Serial(void *parameter) {
  char receiveBuffer[ESP_BUFFER_SIZE] = {0};
  for (;;) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Dequeue
    if (xQueueReceive(send_to_Serial_queue, receiveBuffer, portMAX_DELAY) == pdTRUE) {
      // Write msg to Serial port
      Serial.print("JSON>> ");
      Serial.println(receiveBuffer);
      Serial.flush();  // Wait for TX to complete
      // clear buffer
      for (size_t _i = 0; _i < ESP_BUFFER_SIZE; _i++) {
        receiveBuffer[_i] = 0;
      }
    }
  }
}

/// @brief ESP-Now message sent callback
/// @param mac_addr MAC address of the sending device
/// @param status True on succesfull send
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) Serial.println("Delivery Fail");
}

/// @brief ESP-NOW message received callback
///        Queues ESP-NOW message to Serial handling queue 'send_to_Serial_queue'
/// @param mac MAC address of the sending device
/// @param incomingData Data from ESP-NOW packet
/// @param len Length of incoming data
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  char buffer[ESP_BUFFER_SIZE] = {0};
  if ( len <= ESP_BUFFER_SIZE ) {
    memcpy(buffer, incomingData, len);
    if (xQueueSend(send_to_Serial_queue, (void *)buffer, 0) != pdTRUE) {
      Serial.println("Error sending to queue");
    }
  }
}

/// @brief Initialize ESP-NOW interface, register callbacks
///        Call once from setup()
/// @return True upon success, else false
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

/// @brief Calculate uptime & populate uptime buffer for future use
/// @param buffer char * buffer to store uptime information
/// @param size Size of uptime buffer
void uptime(char *buffer, uint8_t size) {
  // Constants for uptime calculations
  static const uint32_t millis_in_day = 1000 * 60 * 60 * 24;
  static const uint32_t millis_in_hour = 1000 * 60 * 60;
  static const uint32_t millis_in_minute = 1000 * 60;

  unsigned long now = millis();
  uint8_t days = now / (millis_in_day);
  uint8_t hours = (now - (days * millis_in_day)) / millis_in_hour;
  uint8_t minutes = (now - (days * millis_in_day) - (hours * millis_in_hour)) / millis_in_minute;
  snprintf(buffer, size, "%2dd%2dh%2dm", days, hours, minutes);
}

void setup() {
  Serial.begin(115200);
  Serial.printf("%s Is Now Woke!\n", DEVICE_NAME);

  // Start esp-now
  if (!initEspNow()) {
    return;
  };

  // Set up queue and tasks
  send_to_Serial_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, ESP_BUFFER_SIZE);
  if (send_to_Serial_queue == NULL) {
    Serial.println("Create Queue failed");
    return;
  }
  xTaskCreate(task_read_ESP_NOW_Write_Serial, "Serial Write Handler", 2048, NULL, 4, &xhandleSerialWriteHandle);
  xTaskCreate(task_read_Serial_Write_ESP_NOW, "Serial Read Handler", 2048, NULL, 4, &xhandleSerialReadHandle);
}

// Send heartbeat in for of serialized JSON doc to serial port
void loop() {
  
  char uptimeBuffer[12];
  uptime(uptimeBuffer, sizeof(uptimeBuffer));

  String jsonsend = "";
  JsonDocument doc;
  doc.clear();
  doc["D"] = DEVICE_NAME;
  doc["T"] = uptimeBuffer;
  doc["S"] = uxTaskGetStackHighWaterMark(xhandleSerialWriteHandle);
  doc["R"] = uxTaskGetStackHighWaterMark(xhandleSerialReadHandle);
  doc["H"] = esp_get_minimum_free_heap_size();

  serializeJson(doc, jsonsend);
  if (xQueueSend(send_to_Serial_queue, jsonsend.c_str(), 0) != pdTRUE) {
    Serial.println("Error sending to queue");
  }
  vTaskDelay(HEARTBEAT / portTICK_PERIOD_MS);
}
