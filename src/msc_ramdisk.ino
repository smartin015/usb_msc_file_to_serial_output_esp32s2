/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

// https://github.com/adafruit/Adafruit_TinyUSB_Arduino/blob/master/examples/MassStorage/msc_ramdisk/msc_ramdisk.ino

#include "Adafruit_TinyUSB.h"

#define RXD2 16
#define TXD2 18

// 8KB is the smallest size that windows allow to mount, but we're only mutating a fraction of that
#define DISK_BLOCK_NUM  172
#define DISK_BLOCK_SIZE 512

// 56k reported size of membory of the clausing mill
#define REPORTED_BLOCK_NUM 172

#include "ramdisk.h"
#include <HardwareSerial.h>

HardwareSerial Serial2(1);

Adafruit_USBD_MSC usb_msc;

// Eject button to demonstrate medium is not ready e.g SDCard is not present
// whenever this button is pressed and hold, it will report to host as not ready
#if defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(ARDUINO_NRF52840_CIRCUITPLAY)
  #define BTN_EJECT   4   // Left Button
  bool activeState = true;

#elif defined(ARDUINO_FUNHOUSE_ESP32S2)
  #define BTN_EJECT   BUTTON_DOWN
  bool activeState = true;

#elif defined PIN_BUTTON1
  #define BTN_EJECT   PIN_BUTTON1
  bool activeState = false;
#endif


// the setup function runs once when you press reset or power the board
void setup() {
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(LED_BUILTIN, OUTPUT);

#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as
  // - mbed rp2040
  TinyUSB_Device_Init(0);
#endif

#ifdef BTN_EJECT
  pinMode(BTN_EJECT, activeState ? INPUT_PULLDOWN : INPUT_PULLUP);
#endif

  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "Mass Storage", "1.0");

  // Set disk size
  usb_msc.setCapacity(REPORTED_BLOCK_NUM, DISK_BLOCK_SIZE);

  // Set callback
  usb_msc.setReadWriteCallback(msc_read_callback, msc_write_callback, msc_flush_callback);
  usb_msc.setStartStopCallback(msc_start_stop_callback);
  usb_msc.setReadyCallback(msc_ready_callback);

  // Set Lun ready (RAM disk is always ready)
  usb_msc.setUnitReady(true);
  usb_msc.begin();
//  while ( !Serial ) delay(10);   // wait for native usb
  Serial2.println("Testing 123");
}

void loop() {
  // nothing to do
  if (queue_len == 0) {
    delay(500);
    Serial2.print(".");
    Serial2.flush();
    digitalWrite(LED_BUILTIN, HIGH);
  }
  while (queue_len > 0) {
    //Serial2.print("Replay block ");
    //Serial2.println(echo_queue[cur_idx]);
    //Serial2.print("Queue len ");
    //Serial2.println(queue_len);
    uint8_t* addr = msc_disk[echo_queue[cur_idx]];
    for (int i = 0; i < sz_queue[cur_idx]; i++) {
      if (i % 128 == 0) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      }
      Serial2.print(char(*(addr+i)));
    }
    cur_idx = (cur_idx + 1) % MAX_QUEUESZ;
    queue_len--;
  }
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_callback(uint32_t lba, void* buffer, uint32_t bufsize) {
  uint8_t const* addr = msc_disk[lba];
  memcpy(buffer, addr, bufsize);
  return bufsize;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_callback(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  //Serial2.print("Write block ");
  //Serial2.println(lba);
  uint8_t* addr = msc_disk[lba];
  memcpy(addr, buffer, bufsize);
  if (lba >= 4) { // Early blocks are filesystem blocks and should be updated accordingly
    echo_queue[(cur_idx + queue_len) % MAX_QUEUESZ] = lba;
    sz_queue[(cur_idx + queue_len) % MAX_QUEUESZ] = bufsize;
    queue_len++;
    Serial2.print("Append block to queue: ");
    Serial2.print(lba);
    Serial2.print(" ");
    Serial2.println(bufsize);
    
  }
  return bufsize; 
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_callback(void) {
  // nothing to do
}

bool msc_start_stop_callback(uint8_t power_condition, bool start, bool load_eject) {
  Serial2.printf("Start/Stop callback: power condition %u, start %u, load_eject %u\n", power_condition, start, load_eject);
  return true;
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool msc_ready_callback(void) {
  #ifdef BTN_EJECT
  // button not active --> medium ready
  return digitalRead(BTN_EJECT) != activeState;
  #else
  return true;
  #endif
}

