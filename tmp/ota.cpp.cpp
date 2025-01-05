#include "soc/rtc_wdt.h"
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

const char *ssid = "TELUS2743";
const char *password = "fgtiw8696a";

int i = 0;

void Task1code(void *pvParameters) {
  Serial.println("Task1 Start");

  rtc_wdt_protect_off(); // Turns off the automatic wdt service
  rtc_wdt_enable();      // Turn it on manually
  rtc_wdt_set_time(RTC_WDT_STAGE0,
                   20000); // Define how long you desire to let dog wait.

  if (i == 0) {
    ArduinoOTA
        .onStart([]() {
          String type;
          if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
          else // U_SPIFFS
            type = "filesystem";

          // NOTE: if updating SPIFFS this would be the place to unmount
          // SPIFFS using SPIFFS.end()
          Serial.println("Start updating " + type);
        })
        .onEnd([]() { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total) {
          Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
          rtc_wdt_feed();
        })
        .onError([](ota_error_t error) {
          Serial.printf("Error[%u]: ", error);
          if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
          else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
          else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
          else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
          else if (error == OTA_END_ERROR)
            Serial.println("End Failed");
        });

    ArduinoOTA.begin();

    Serial.println("OTA Begin");

    i = 1;
  }
  while (1) {
    rtc_wdt_feed();
    ArduinoOTA.handle();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  xTaskCreatePinnedToCore(
      Task1code,        /* Task function. */
      "Task1",          /* name of task. */
      50000,            /* Stack size of task */
      NULL,             /* parameter of the task */
      tskIDLE_PRIORITY, /* priority of the task */
      &Task1,           /* Task handle to keep track of created task */
      0);               /* pin task to core 0 */
  delay(500);
}

void loop() {}