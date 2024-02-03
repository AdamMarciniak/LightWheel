#include "../../../framework-arduinoespressif32/tools/sdk/esp32/include/driver/include/driver/spi_common.h"
#include "../../../framework-arduinoespressif32/tools/sdk/esp32/include/driver/include/driver/spi_master.h"
#include "../../../framework-arduinoespressif32/tools/sdk/esp32/include/heap/include/esp_heap_caps.h"
#include <Arduino.h>
#include <Chrono.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <ledArray.h>

/* INCLUDE ESP2SOTA LIBRARY */
#include <AsyncTCP.h>
#include <ESP2SOTA.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

// Flash datasheet:
// https://www.winbond.com/resource-files/w25q80dv%20dl_revh_10022015.pdf
#define CMD_WRITE_ENABLE 0X06
#define CMD_READ_STATUS_REGISTER_1 0X05
#define CMD_PG_PROGRAM 0X02
#define CMD_SECTOR_ERASE 0X20 // 4KB
#define CMD_CHIP_ERASE 0XC7
#define CMD_READ_DATA 0X03
#define CMD_READ_DATA_FAST 0X0B
#define CMD_READ_JEDEC_ID 0X9F
#define BUSY_BIT 0B00000001

#define NUM_ARMS 4
#define NUM_LED_PER_ARM 42
#define ANGLE_RESOLUTION_DEG (int)1
#define NUM_BYTES_PER_LED 4
#define BYTES_PER_ARM (NUM_LED_PER_ARM * NUM_BYTES_PER_LED)

#define BYTES_PER_FRAME                                                        \
  (NUM_LED_PER_ARM * (360 / ANGLE_RESOLUTION_DEG) * NUM_BYTES_PER_LED)

#define LED_TX_END_SIZE 4
#define LED_TX_START_SIZE 4
#define ALL_ARM_DATA_SIZE (BYTES_PER_ARM * NUM_ARMS) + LED_TX_END_SIZE
#define LED_TX_LENGTH (ALL_ARM_DATA_SIZE * 8)
#define ARM_BETWEEN_SIZE (BYTES_PER_FRAME / NUM_ARMS)

#define MAX_READ_SIZE 4092
#define ACTUAL_FRAME_DATA_SIZE                                                 \
  ((int)(BYTES_PER_FRAME / MAX_READ_SIZE) + 1) * MAX_READ_SIZE
#define NUM_READ_PAGES (int)(ACTUAL_FRAME_DATA_SIZE / MAX_READ_SIZE)

#define ARM_2_BETWEEN ARM_BETWEEN_SIZE * 2
#define ARM_3_BETWEEN ARM_BETWEEN_SIZE * 3

#define MAX_FLASH_WRITE_BYTES 256
#define FLASH_WRITE_FRAME_PAGE_NUM                                             \
  (int)(ACTUAL_FRAME_DATA_SIZE / MAX_FLASH_WRITE_BYTES + 1) *                  \
      MAX_FLASH_WRITE_BYTES

#define LED_HOST VSPI_HOST
#define FLASH_HOST HSPI_HOST

enum State {
  DISPLAY_LEDS,
  FIRST_STOP,
  IDLE,
};

State state = FIRST_STOP;

const char *ssid = "ESP2SOTA";
const char *password = "adamadam123";

unsigned long before = 0;
unsigned long after = 0;

unsigned long bef = 0;

unsigned long lastMagnetTime = 0;
unsigned long nextMagnetTime = 1000;

unsigned long lastMagnetTime1 = 0;
unsigned long nextMagnetTime1 = 1000;
unsigned long rotTime = 1000;
long armPos = 0;

unsigned long timeStopped = 0;

Chrono flashReadChrono;
Chrono ledChrono(Chrono::MICROS);
Chrono spinCheckChrono;

Chrono webSerialChrono;
Chrono webSerialChrono1;

void IRAM_ATTR magnetISR() {
  lastMagnetTime1 = nextMagnetTime1;
  nextMagnetTime1 = micros();
  state = DISPLAY_LEDS;
  spinCheckChrono.restart();
  if ((nextMagnetTime1 - lastMagnetTime1) < 20000) {
    return;
  }
  lastMagnetTime = lastMagnetTime1;
  nextMagnetTime = nextMagnetTime1;
  rotTime = nextMagnetTime - lastMagnetTime;
  armPos = 0;
  ledChrono.restart();
}

boolean isSpinning = false;

spi_bus_config_t spiConfigLED = {
    .mosi_io_num = 23,
    .sclk_io_num = 18,
    .max_transfer_sz = 0,
    .flags = SPICOMMON_BUSFLAG_MASTER, ///< Abilities of bus to be checked by
                                       ///< the driver. Or-ed value of
                                       ///< ``SPICOMMON_BUSFLAG_*`` flags.
                                       //.intr_flags = ESP_INTR_FLAG_IRAM
};

spi_bus_config_t spiConfigFlash = {
    .mosi_io_num = 13,
    .miso_io_num = 12,
    .sclk_io_num = 14,
    .flags = SPICOMMON_BUSFLAG_MASTER, ///< Abilities of bus to be checked by
                                       ///< the driver. Or-ed value of
                                       ///< ``SPICOMMON_BUSFLAG_*`` flags.
                                       //.intr_flags = ESP_INTR_FLAG_IRAM
};

DRAM_ATTR int queueStatus = 2;
DRAM_ATTR int queueStatusFlash = 2;
DRAM_ATTR boolean isFlashBusy = false;
DRAM_ATTR boolean ledReady = true;

int color = 1;

unsigned long milPre = 0;
unsigned long milPost = 0;
// Set to be in IRAM
// Set some bit and make that in IRAM too.
void IRAM_ATTR callBackPreLED(spi_transaction_t *trans) { ledReady = false; };

void IRAM_ATTR callBackPostLED(spi_transaction_t *trans) { ledReady = true; };

void IRAM_ATTR callBackPreFlash(spi_transaction_t *trans) {
  isFlashBusy = true;
};

void IRAM_ATTR callBackPostFlash(spi_transaction_t *trans) {
  isFlashBusy = false;
};

const spi_device_interface_config_t deviceConfigLED = {
    .command_bits = 8,
    .address_bits = 24,
    .dummy_bits = 0,
    .mode = 0,
    .clock_speed_hz = 15000000,
    .queue_size = 1,
    .pre_cb = callBackPreLED,
    .post_cb = callBackPostLED};

const spi_device_interface_config_t deviceConfigFlash = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 0,
    .clock_speed_hz = 45000000,
    .spics_io_num = 15,
    .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY,
    .queue_size = 1,
    .pre_cb = callBackPreFlash,
    .post_cb = callBackPostFlash};

esp_err_t deviceResult;
esp_err_t transactionResult;
spi_device_handle_t deviceHandleLED;
spi_device_handle_t deviceHandleFlashErase;
spi_device_handle_t deviceHandleFlashWriteEnable;
spi_device_handle_t deviceHandleFlashWrite;

uint8_t *frameBuffers[360];

uint8_t *armBuffer;
uint8_t *darkBuffer;
uint8_t *frameData;

uint8_t *writePageData;
uint8_t *oneLEDData;
uint8_t *flashBuffer;
uint8_t *flashReadBuffer2;

uint8_t *bufferToRead = flashBuffer;
uint8_t *bufferToWriteFrom = flashBuffer;
uint8_t *testReadBuffer;

uint8_t *busyBuffer;

void writeData(spi_device_handle_t deviceHandle, size_t lengthInBits,
               uint8_t *tx_buffer, uint64_t address) {
  spi_transaction_t transactionFlashWrite{.flags = SPI_TRANS_VARIABLE_CMD |
                                                   SPI_TRANS_VARIABLE_ADDR,
                                          .cmd = CMD_PG_PROGRAM,
                                          .addr = address,
                                          .length = lengthInBits,
                                          .rxlength = 0,
                                          .tx_buffer = tx_buffer};
  spi_transaction_ext_t transactionFlashWriteExt{.base = transactionFlashWrite,
                                                 .command_bits = 8,
                                                 .address_bits = 24,
                                                 .dummy_bits = 0};
  spi_device_transmit(deviceHandle,
                      (spi_transaction_t *)&transactionFlashWriteExt);

  delay(10);
}

void writeEnable(spi_device_handle_t deviceHandle) {
  spi_transaction_t transactionFlashWrite{
      .flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
      .cmd = CMD_WRITE_ENABLE,
      .length = 0,
      .rxlength = 0,
  };
  spi_transaction_ext_t transactionFlashWriteExt{.base = transactionFlashWrite,
                                                 .command_bits = 8,
                                                 .address_bits = 0,
                                                 .dummy_bits = 0};
  spi_device_transmit(deviceHandle,
                      (spi_transaction_t *)&transactionFlashWriteExt);
  delay(10);
}

void writeDisable(spi_device_handle_t deviceHandle) {
  spi_transaction_t transactionFlashWrite{
      .flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
      .cmd = 0X04,
      .length = 0,
      .rxlength = 0,
  };
  spi_transaction_ext_t transactionFlashWriteExt{.base = transactionFlashWrite,
                                                 .command_bits = 8,
                                                 .address_bits = 0,
                                                 .dummy_bits = 0};
  spi_device_transmit(deviceHandle,
                      (spi_transaction_t *)&transactionFlashWriteExt);
  delay(10);
}

void waitBusy(spi_device_handle_t deviceHandle) {
  int busy = 1;
  busyBuffer[0] = 0X00;
  while (busy == 1) {
    spi_transaction_t transactionReadStatusRegister{
        .flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
        .cmd = CMD_READ_STATUS_REGISTER_1,
        .length = 0,
        .rxlength = 8,
        .rx_buffer = busyBuffer};
    spi_transaction_ext_t transactionReadStatusRegisterExt{
        .base = transactionReadStatusRegister,
        .command_bits = 8,
        .address_bits = 0,
        .dummy_bits = 0};
    spi_device_transmit(deviceHandle,
                        (spi_transaction_t *)&transactionReadStatusRegisterExt);

    Serial.println(busyBuffer[0], BIN);

    if (busyBuffer[0] & 0X01) {
      busy = 1;
      Serial.println("BUSY");
    } else {
      busy = 0;
    }

    delay(100);
  }
}

void chipErase(spi_device_handle_t deviceHandle) {
  Serial.println("Erasing chip");
  spi_transaction_t transactionFlashErase{
      .flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR,
      .cmd = CMD_CHIP_ERASE,
      .addr = 0,
      .length = 0,
      .rxlength = 0,
      .user = 0,
  };
  spi_transaction_ext_t transactionFlashEraseExt{.base = transactionFlashErase,
                                                 .command_bits = 8,
                                                 .address_bits = 0,
                                                 .dummy_bits = 0};
  spi_device_transmit(deviceHandle,
                      (spi_transaction_t *)&transactionFlashEraseExt);
  delay(10);
}

uint8_t *testBuffer;

uint8_t chunk[3];
int chunkIdx = 0;
boolean wifiFrameReady = false;
int frameIndex = 0;
int num = 0;
boolean firstDigit = false;
void recvMsg(uint8_t *data, size_t len) {
  WebSerial.println("Received Data...");
  for (int i = 0; i < len; i++) {
    WebSerial.println(data[i]);
    WebSerial.println();

    if (data[i] != ',') {
      chunk[chunkIdx] = data[i] - '0';
      chunkIdx += 1;

    } else {
      // we have full number
      for (int j = 0; j < chunkIdx; j += 1) {
        WebSerial.print(chunk[j]);
      }
      chunkIdx = 0;
    }
  }
}

int counter = 0;

WebServer server(80);
AsyncWebServer asyncServer(81);
void wifiTask(void *parameter) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  delay(1000);
  IPAddress IP = IPAddress(10, 10, 10, 1);
  IPAddress NMask = IPAddress(255, 255, 255, 0);
  WiFi.softAPConfig(IP, IP, NMask);
  IPAddress myIP = WiFi.softAPIP();
  ESP2SOTA.begin(&server);
  server.begin();

  /* SETUP YOR WEB OWN ENTRY POINTS */
  server.on("/myurl", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "Hello there. THIS HAS CHANGED!!");
  });

  WebSerial.msgCallback(recvMsg);
  WebSerial.begin(&asyncServer);
  asyncServer.begin();

  flashReadBuffer2 = static_cast<uint8_t *>(
      heap_caps_malloc(ACTUAL_FRAME_DATA_SIZE, MALLOC_CAP_DMA));

  for (long i = 0; i < ACTUAL_FRAME_DATA_SIZE; i += 1) {
    flashReadBuffer2[i] = 0X00;
  }

  for (;;) {
    server.handleClient();
  }
}

long ledCounter = 0;
long ledIndex = 0;

int i = 0;

boolean newFrame = true;
uint64_t readPageNum = 0;
int frameNum = 1;
uint64_t frameOffset = 0;
uint64_t address = 0;

void ledTask(void *parameter) {

  pinMode(5, INPUT_PULLUP);
  bufferToWriteFrom = flashBuffer;
  esp_err_t resultLEDInit = spi_bus_initialize(LED_HOST, &spiConfigLED, 2);
  ESP_ERROR_CHECK(resultLEDInit);
  esp_err_t deviceResultLED =
      spi_bus_add_device(LED_HOST, &deviceConfigLED, &deviceHandleLED);
  ESP_ERROR_CHECK(deviceResultLED);
  esp_err_t resultFlashInit =
      spi_bus_initialize(FLASH_HOST, &spiConfigFlash, 1);
  ESP_ERROR_CHECK(resultFlashInit);
  // esp_err_t deviceResultFlash = spi_bus_add_device(FLASH_HOST,
  // &deviceConfigFlashRead, &deviceHandleFlashRead);
  // ESP_ERROR_CHECK(deviceResultLED);
  esp_err_t deviceResultFlashWrite = spi_bus_add_device(
      FLASH_HOST, &deviceConfigFlash, &deviceHandleFlashWrite);
  ESP_ERROR_CHECK(deviceResultFlashWrite);

  flashBuffer = static_cast<uint8_t *>(
      heap_caps_malloc(ACTUAL_FRAME_DATA_SIZE, MALLOC_CAP_DMA));
  // armBuffer = static_cast<uint8_t *>(heap_caps_malloc(676, MALLOC_CAP_DMA));
  armBuffer = static_cast<uint8_t *>(heap_caps_malloc(716, MALLOC_CAP_DMA));
  darkBuffer = static_cast<uint8_t *>(heap_caps_malloc(716, MALLOC_CAP_DMA));
  testBuffer = static_cast<uint8_t *>(heap_caps_malloc(20, MALLOC_CAP_DMA));

  for (long i = 0; i < 716; i += 4) {
    darkBuffer[i] = 0B11100001;
    darkBuffer[i + 1] = 0x0;
    darkBuffer[i + 2] = 0x0;
    darkBuffer[i + 3] = 50;
  }

  darkBuffer[716 - 4] = 0XFF;
  darkBuffer[716 - 3] = 0XFF;
  darkBuffer[716 - 2] = 0XFF;
  darkBuffer[716 - 1] = 0XFF;

  for (long i = 0; i < ACTUAL_FRAME_DATA_SIZE; i += 1) {
    flashBuffer[i] = 0X00;
  }

  for (long i = 0; i < ACTUAL_FRAME_DATA_SIZE - 4; i += 4) {
    ledIndex = ledCounter * 3;
    if (ledIndex >= 45360) {
      ledCounter = 0;
      ledIndex = ledCounter * 3;
    }
    flashBuffer[i] = 0B11100001;
    // flashReadBuffer[i + 1] = ledArray[ledIndex + 2];
    // flashReadBuffer[i + 2] = ledArray[ledIndex + 1];
    // flashReadBuffer[i + 3] = ledArray[ledIndex];
    flashBuffer[i + 1] = 150;
    flashBuffer[i + 2] = 150;
    flashBuffer[i + 3] = 150;
    ledCounter += 1;
  }

  // armBuffer[676 - 4] = 0XFF;
  // armBuffer[676 - 3] = 0XFF;
  // armBuffer[676 - 2] = 0XFF;
  // armBuffer[676 - 1] = 0XFF;

  for (long i = 0; i < 716; i += 4) {
    armBuffer[i] = 0B11100001;
    armBuffer[i + 1] = 0x0;
    armBuffer[i + 2] = 120;
    armBuffer[i + 3] = 0x0;
  }

  armBuffer[716 - 4] = 0XFF;
  armBuffer[716 - 3] = 0XFF;
  armBuffer[716 - 2] = 0XFF;
  armBuffer[716 - 1] = 0XFF;

  Serial.println("Done");
  lastMagnetTime = micros();
  nextMagnetTime = micros();

  attachInterrupt(5, magnetISR, FALLING);

  for (;;) {
    if (webSerialChrono.hasPassed(1000)) {
      webSerialChrono.restart();
      Serial.println("HELLO");
    }
    switch (state) {
    case (FIRST_STOP):
      // Write all dark to turn off lights.
      if (ledReady == true) {
        spi_transaction_t transactionLED{
            .cmd = 0X00,
            .addr = 0X00,
            .length = 5728,
            .rxlength = 0,
            .user = 0,
            .tx_buffer = darkBuffer,
        };
        spi_device_queue_trans(deviceHandleLED, &transactionLED, portMAX_DELAY);
        state = IDLE;
      }

      break;

    case (DISPLAY_LEDS):

      if (spinCheckChrono.hasPassed(2000)) {
        state = FIRST_STOP;
        break;
      }

      if (ledChrono.hasPassed((long)(rotTime / 360))) {
        ledChrono.restart();
        armPos += 1;
        if (armPos >= 360) {
          armPos = 0;
        }
        if (ledReady == true) {
          // Use this below for all 4 arms
          armPos = 0;
          memcpy(armBuffer, &bufferToWriteFrom[armPos * BYTES_PER_ARM],
                 BYTES_PER_ARM);
          memcpy(
              &armBuffer[BYTES_PER_ARM],
              &bufferToWriteFrom[(armPos * BYTES_PER_ARM + ARM_BETWEEN_SIZE) %
                                 BYTES_PER_FRAME],
              BYTES_PER_ARM);
          memcpy(&armBuffer[BYTES_PER_ARM * 2],
                 &bufferToWriteFrom[(armPos * BYTES_PER_ARM + ARM_2_BETWEEN) %
                                    BYTES_PER_FRAME],
                 BYTES_PER_ARM);
          memcpy(&armBuffer[BYTES_PER_ARM * 3],
                 &bufferToWriteFrom[(armPos * BYTES_PER_ARM + ARM_3_BETWEEN) %
                                    BYTES_PER_FRAME],
                 BYTES_PER_ARM);

          spi_transaction_t transactionLED{
              .cmd = 0X00,
              .addr = 0X00,
              // .length = 5408,
              .length = 5728,
              .rxlength = 0,
              .user = 0,
              .tx_buffer = armBuffer,
          };
          spi_device_queue_trans(deviceHandleLED, &transactionLED,
                                 portMAX_DELAY);
        }
      }
      break;
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  xTaskCreatePinnedToCore(wifiTask,   /* Function to implement the task */
                          "wifiTask", /* Name of the task */
                          10000,      /* Stack size in words */
                          NULL,       /* Task input parameter */
                          0,          /* Priority of the task */
                          NULL,       /* Task handle. */
                          0);         /* Core where the task should run */

  xTaskCreatePinnedToCore(ledTask,   /* Function to implement the task */
                          "ledTask", /* Name of the task */
                          10000,     /* Stack size in words */
                          NULL,      /* Task input parameter */
                          2,         /* Priority of the task */
                          NULL,      /* Task handle. */
                          1);        /* Core where the task should run */
}

void loop() { vTaskDelete(NULL); }