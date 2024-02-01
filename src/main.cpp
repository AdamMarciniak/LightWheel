#include "../../../framework-arduinoespressif32/tools/sdk/esp32/include/driver/include/driver/spi_common.h"
#include "../../../framework-arduinoespressif32/tools/sdk/esp32/include/driver/include/driver/spi_master.h"
#include "../../../framework-arduinoespressif32/tools/sdk/esp32/include/heap/include/esp_heap_caps.h"
#include <Arduino.h>
#include <Chrono.h>

// File to store LED data
#include <ledArray.h>

// Flash datasheet:
// https://www.winbond.com/resource-files/w25q80dv%20dl_revh_10022015.pdf
// Flash registers
#define CMD_WRITE_ENABLE 0X06
#define CMD_READ_STATUS_REGISTER_1 0X05
#define CMD_PG_PROGRAM 0X02
#define CMD_SECTOR_ERASE 0X20 // 4KB
#define CMD_CHIP_ERASE 0XC7
#define CMD_READ_DATA 0X03
#define CMD_READ_DATA_FAST 0X0B
#define CMD_READ_JEDEC_ID 0X9F
#define BUSY_BIT 0B00000001

// Number of arms around wheel
#define NUM_ARMS 4
#define NUM_LED_PER_ARM 42

#define ANGLE_RESOLUTION_DEG (int)1
// Hard set in LED datasheet
#define NUM_BYTES_PER_LED 4
#define BYTES_PER_ARM (NUM_LED_PER_ARM * NUM_BYTES_PER_LED)

// BYTES to store one image
#define BYTES_PER_FRAME                                                        \
  (NUM_LED_PER_ARM * (360 / ANGLE_RESOLUTION_DEG) * NUM_BYTES_PER_LED)

// LED transaction config
#define LED_TX_END_SIZE 4
#define LED_TX_START_SIZE 4
#define ALL_ARM_DATA_SIZE (BYTES_PER_ARM * NUM_ARMS) + LED_TX_END_SIZE
#define LED_TX_LENGTH (ALL_ARM_DATA_SIZE * 8)

// Size of data between arms. Used to offset each arm position in data array
#define ARM_OFFSET_SIZE (BYTES_PER_FRAME / NUM_ARMS)

#define MAX_READ_SIZE 4092
#define ACTUAL_FRAME_DATA_SIZE                                                 \
  ((int)(BYTES_PER_FRAME / MAX_READ_SIZE) + 1) * MAX_READ_SIZE
#define NUM_READ_PAGES (int)(ACTUAL_FRAME_DATA_SIZE / MAX_READ_SIZE)

// Offsets
#define ARM_2_OFFSET ARM_OFFSET_SIZE * 2
#define ARM_3_OFFSET ARM_OFFSET_SIZE * 3

#define MAX_FLASH_WRITE_BYTES 256
#define FLASH_WRITE_FRAME_PAGE_NUM                                             \
  (int)(ACTUAL_FRAME_DATA_SIZE / MAX_FLASH_WRITE_BYTES + 1) *                  \
      MAX_FLASH_WRITE_BYTES

#define LED_HOST VSPI_HOST
#define FLASH_HOST HSPI_HOST

#define MAGNET_SENSE_PIN 5

enum State {
  DISPLAY_LEDS,
  FIRST_STOP,
  IDLE,
};

// Add back for magnet operation
// State state = FIRST_STOP;
State state = DISPLAY_LEDS;

unsigned long lastMagnetTime = 0;
unsigned long nextMagnetTime = 1000;

unsigned long lastMagnetTime1 = 0;
unsigned long nextMagnetTime1 = 1000;
unsigned long rotTime = 1000;
long armPosition = 0;
int frameNum = 0;

unsigned long timeStopped = 0;

Chrono flashReadChrono;
Chrono ledChrono(Chrono::MICROS);
Chrono frameChrono;

Chrono spinCheckChrono;

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
  armPosition = 0;
  ledChrono.restart();
}

boolean isSpinning = false;

spi_bus_config_t spiConfigLED = {
    .mosi_io_num = 23,
    .sclk_io_num = 18,
    .max_transfer_sz = 0,
    .flags = SPICOMMON_BUSFLAG_MASTER,
};

spi_bus_config_t spiConfigFlash = {
    .mosi_io_num = 13,
    .miso_io_num = 12,
    .sclk_io_num = 14,
    .flags = SPICOMMON_BUSFLAG_MASTER,
};

DRAM_ATTR int queueStatus = 2;
DRAM_ATTR int queueStatusFlash = 2;
DRAM_ATTR boolean flashBusy = false;
DRAM_ATTR boolean isSPIReady = true;

int color = 1;

unsigned long milPre = 0;
unsigned long milPost = 0;

// Before we write to LED's, set ready to false to prevent more writes
void IRAM_ATTR callBackPreLED(spi_transaction_t *trans) { isSPIReady = false; };

// Once writing is done, allow writes
void IRAM_ATTR callBackPostLED(spi_transaction_t *trans) { isSPIReady = true; };

// Same with flash
void IRAM_ATTR callBackPreFlash(spi_transaction_t *trans) { flashBusy = true; };
void IRAM_ATTR callBackPostFlash(spi_transaction_t *trans) {
  flashBusy = false;
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

uint8_t *frameBuffer1;
uint8_t *frameBuffer2;

uint8_t *bufferToRead = frameBuffer1;
uint8_t *testReadBuffer;

uint8_t *busyBuffer;

// Writes to flash
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

// Wait until flash is not busy
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

long ledCounter = 0;
long ledIndex = 0;

void setup() {
  Serial.begin(115200);

  pinMode(MAGNET_SENSE_PIN, INPUT_PULLUP);

  // Init LED and flash stuff and handle errors
  ESP_ERROR_CHECK(spi_bus_initialize(LED_HOST, &spiConfigLED, 2));
  ESP_ERROR_CHECK(
      spi_bus_add_device(LED_HOST, &deviceConfigLED, &deviceHandleLED));
  ESP_ERROR_CHECK(spi_bus_initialize(FLASH_HOST, &spiConfigFlash, 1));
  ESP_ERROR_CHECK(spi_bus_add_device(FLASH_HOST, &deviceConfigFlash,
                                     &deviceHandleFlashWrite));

  // Init memory buffers

  // Holds data of whole frame
  frameBuffer1 = static_cast<uint8_t *>(
      heap_caps_malloc(ACTUAL_FRAME_DATA_SIZE, MALLOC_CAP_DMA));

  frameBuffer2 = static_cast<uint8_t *>(
      heap_caps_malloc(ACTUAL_FRAME_DATA_SIZE, MALLOC_CAP_DMA));

  // Flag for knowing when flash is busy
  busyBuffer = static_cast<uint8_t *>(heap_caps_malloc(1, MALLOC_CAP_DMA));

  // Holds data for current state of arms. A slice of frame data
  armBuffer = static_cast<uint8_t *>(heap_caps_malloc(676, MALLOC_CAP_DMA));

  // Holds data for arms when dark
  darkBuffer = static_cast<uint8_t *>(heap_caps_malloc(716, MALLOC_CAP_DMA));

  for (long i = 0; i < ACTUAL_FRAME_DATA_SIZE; i += 1) {
    frameBuffer1[i] = 0X00;
  }

  ledCounter = 0;
  ledIndex = 0;
  for (long i = 0; i < ACTUAL_FRAME_DATA_SIZE - 4; i += 4) {
    ledIndex = ledCounter * 3;
    if (ledIndex >= 45360) {
      ledCounter = 0;
      ledIndex = ledCounter * 3;
    }
    frameBuffer1[i] = 0B11100001;
    frameBuffer1[i + 1] = ledArray2[ledIndex + 2];
    frameBuffer1[i + 2] = ledArray2[ledIndex + 1];
    frameBuffer1[i + 3] = ledArray2[ledIndex];
    ledCounter += 1;
  }

  ledCounter = 0;
  ledIndex = 0;
  for (long i = 0; i < ACTUAL_FRAME_DATA_SIZE - 4; i += 4) {
    ledIndex = ledCounter * 3;
    if (ledIndex >= 45360) {
      ledCounter = 0;
      ledIndex = ledCounter * 3;
    }
    frameBuffer2[i] = 0B11100001;
    frameBuffer2[i + 1] = ledArray3[ledIndex + 2];
    frameBuffer2[i + 2] = ledArray3[ledIndex + 1];
    frameBuffer2[i + 3] = ledArray3[ledIndex];
    ledCounter += 1;
  }

  armBuffer[676 - 4] = 0XFF;
  armBuffer[676 - 3] = 0XFF;
  armBuffer[676 - 2] = 0XFF;
  armBuffer[676 - 1] = 0XFF;

  // Buffer of data to turn off LED's
  for (long i = 0; i < 716; i += 4) {
    darkBuffer[i] = 0B11100000;
    darkBuffer[i + 1] = 0x0;
    darkBuffer[i + 2] = 0x0;
    darkBuffer[i + 3] = 0x0;
  }

  darkBuffer[716 - 4] = 0XFF;
  darkBuffer[716 - 3] = 0XFF;
  darkBuffer[716 - 2] = 0XFF;
  darkBuffer[716 - 1] = 0XFF;

  lastMagnetTime = micros();
  nextMagnetTime = micros();

  attachInterrupt(5, magnetISR, FALLING);
}

void copyMemoryToArms(uint8_t *inputBuffer, long armPosition) {
  memcpy(armBuffer, &inputBuffer[armPosition * BYTES_PER_ARM], BYTES_PER_ARM);
  memcpy(&armBuffer[BYTES_PER_ARM],
         &inputBuffer[(armPosition * BYTES_PER_ARM + ARM_OFFSET_SIZE) %
                      BYTES_PER_FRAME],
         BYTES_PER_ARM);
  memcpy(&armBuffer[BYTES_PER_ARM * 2],
         &inputBuffer[(armPosition * BYTES_PER_ARM + ARM_2_OFFSET) %
                      BYTES_PER_FRAME],
         BYTES_PER_ARM);
  memcpy(&armBuffer[BYTES_PER_ARM * 3],
         &inputBuffer[(armPosition * BYTES_PER_ARM + ARM_3_OFFSET) %
                      BYTES_PER_FRAME],
         BYTES_PER_ARM);
}

void loop() {

  switch (state) {

  case (FIRST_STOP):
    // Write all dark to turn off lights.
    if (isSPIReady) {
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
  case (IDLE):
    break;

  case (DISPLAY_LEDS):
    // If magnet has not been hit in some time, stop display
    // Add back for magnet operation
    // if (spinCheckChrono.hasPassed(4000)) {
    //   state = FIRST_STOP;
    //   break;
    // }

    if (frameChrono.hasPassed(3000)) {
      frameChrono.restart();
      if (frameNum == 0) {
        frameNum = 1;
        return;
      }
      if (frameNum == 1) {
        frameNum = 0;
        return;
      }
    }

    // Change this to use magnet  for display switching or  time based
    // if (ledChrono.hasPassed((long)(rotTime / 360))) {
    if (ledChrono.hasPassed((long)(1500000 / 360))) {

      ledChrono.restart();
      armPosition += 1;
      if (armPosition >= 360 / ANGLE_RESOLUTION_DEG) {
        armPosition = 0;
      }
      if (isSPIReady) {
        // Use this below for all 4 arms
        // This take like 3 microseconds. No need to optimize
        if (frameNum == 0) {
          copyMemoryToArms(frameBuffer1, armPosition);
        } else {
          copyMemoryToArms(frameBuffer2, armPosition);
        }

        spi_transaction_t transactionLED{
            .cmd = 0X00,
            .addr = 0X00,
            .length = 5408,
            .rxlength = 0,
            .user = 0,
            .tx_buffer = armBuffer,
        };
        spi_device_queue_trans(deviceHandleLED, &transactionLED, portMAX_DELAY);
      }
    }

    break;
  }
}
