

#include <WebServer.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// Access Point configuration
const char *ap_ssid = "ESP32_AP";
const char *ap_password = "12345678"; // minimum 8 characters
const IPAddress local_ip(192, 168, 1, 1);
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);

// Buffer size constants
const int BUFFER_SIZE = 45360;

// Global variables
uint8_t sharedBuffer[BUFFER_SIZE];
SemaphoreHandle_t bufferMutex = NULL;
WebServer server(80);

// Flag to indicate new data
volatile bool newDataAvailable = false;

// Task handles
TaskHandle_t Task1;
TaskHandle_t Task2;

unsigned int dataSize = 0;

void handlePost() {
  if (server.hasArg("plain")) {
    String data = server.arg("plain");

    // Take mutex before accessing shared buffer
    if (xSemaphoreTake(bufferMutex, portMAX_DELAY)) {
      // Parse incoming data and fill buffer
      dataSize = data.length();
      for (int i = 0; i < BUFFER_SIZE && i < data.length(); i++) {
        sharedBuffer[i] = (uint8_t)data[i];
      }
      xSemaphoreGive(bufferMutex);
      newDataAvailable = true;
    }

    server.send(200, "text/plain", "Data received");
  } else {
    server.send(400, "text/plain", "No data received");
  }
}

// Core 0: WiFi and Web Server task
void Task1code(void *parameter) {
  WiFi.softAP(ap_ssid, ap_password);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  Serial.println("Access Point Started");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/data", HTTP_POST, handlePost);
  server.begin();

  for (;;) {
    server.handleClient();
    vTaskDelay(1); // Prevent watchdog trigger
  }
}

// Core 1: Display task
void Task2code(void *parameter) {
  for (;;) {
    if (newDataAvailable) {

      if (xSemaphoreTake(bufferMutex, portMAX_DELAY)) {
        newDataAvailable = false;
        Serial.print("data Size: ");
        Serial.println(dataSize);

        // Display or process the buffer data
        Serial.println("New data received:");
        for (int i = 0; i < BUFFER_SIZE; i++) {
          Serial.print(sharedBuffer[i]);
          Serial.print(" ");
          if ((i + 1) % 100 == 0)
            Serial.println(); // New line every 20 numbers
        }
        Serial.println();

        xSemaphoreGive(bufferMutex);
      }
    }
    vTaskDelay(10); // Reduce CPU usage
  }
}

void setup() {
  Serial.begin(115200);

  // Create mutex
  bufferMutex = xSemaphoreCreateMutex();

  // Create tasks for two cores
  xTaskCreatePinnedToCore(Task1code, // Function to implement the task
                          "Task1",   // Name of the task
                          50000,     // Stack size in words
                          NULL,      // Task input parameter
                          2,         // Priority of the task
                          &Task1,    // Task handle
                          0          // Core where the task should run
  );

  xTaskCreatePinnedToCore(Task2code, // Function to implement the task
                          "Task2",   // Name of the task
                          50000,     // Stack size in words
                          NULL,      // Task input parameter
                          1,         // Priority of the task
                          &Task2,    // Task handle
                          1          // Core where the task should run
  );
}

void loop() {
  // Empty loop as tasks are handling the work
}