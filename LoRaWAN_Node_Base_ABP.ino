/*  
 * || Project:          This initial implementation is part of the project: 
 * || "IoT Energy Meter with C/C++/FreeRTOS, Java/Spring, TypeScript/Angular and Dart/Flutter.";
 * || About:            End-to-end implementation of a LoRaWAN network for monitoring electrical quantities;
 * || Version:          1.0;
 * || Backend Mote:     ATmega328P/ESP32/ESP8266/ESP8285/STM32;
 * || Radios:           RFM95w and LoRaWAN EndDevice Radioenge Module: RD49C;
 * || Sensors:          Peacefair PZEM-004T 3.0 Version TTL-RTU kWh Meter;
 * || Backend API:      Java with Framework: Spring Boot;
 * || LoRaWAN Stack:    MCCI Arduino LoRaWAN Library (LMiC: LoRaWAN-MAC-in-C) version 3.0.99;
 * || Activation mode:  Activation by Personalization (ABP) or Over-the-Air Activation (OTAA);
 * || Author:           Adail dos Santos Silva
 * || E-mail:           adail101@hotmail.com
 * || WhatsApp:         +55 89 9 9412-9256
 * || 
 * || WARNINGS:
 * || Permission is hereby granted, free of charge, to any person obtaining a copy of
 * || this software and associated documentation files (the “Software”), to deal in
 * || the Software without restriction, including without limitation the rights to
 * || use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * || the Software, and to permit persons to whom the Software is furnished to do so,
 * || subject to the following conditions:
 * || 
 * || The above copyright notice and this permission notice shall be included in all
 * || copies or substantial portions of the Software.
 * || 
 * || THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * || IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * || FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * || COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * || IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * || CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 */

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

/* Includes. */
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

/* Definitions. */
#define RADIO_SCLK_PORT         18
#define RADIO_MISO_PORT         19
#define RADIO_MOSI_PORT         23
#define RADIO_NSS_PORT          5
#define RADIO_RESET_PORT        14
#define RADIO_DIO_0_PORT        34
#define RADIO_DIO_1_PORT        35 /* Note: not really used on this board. */
#define RADIO_DIO_2_PORT        39

/* Credentials. */
// CHIRPSTACK - CS (8 at 15 + 65 channels):
/* big-endian */  // 59bfc689824bda7d0ef4a51e56cfc2af
static const u1_t PROGMEM APPSKEY[16] = { 0x59, 0xBF, 0xC6, 0x89, 0x82, 0x4B, 0xDA, 0x7D, 0x0E, 0xF4, 0xA5, 0x1E, 0x56, 0xCF, 0xC2, 0xAF };

/* big-endian */  // a955390e486a6fe4a2575ae87ccdf299
static const PROGMEM u1_t NWKSKEY[16] = { 0xA9, 0x55, 0x39, 0x0E, 0x48, 0x6A, 0x6F, 0xE4, 0xA2, 0x57, 0x5A, 0xE8, 0x7C, 0xCD, 0xF2, 0x99 };

/* big-endian */                         // 23c4e71c
static const u4_t DEVADDR = 0x23c4e71c;  // <-- Change this address for every node!

//// THE THINGS NETWORK - TTN (8 at 15 + 65 channels):
///* big-endian */ // a59b565ed0c708cc2bf27d8b7140ed90
//static const u1_t PROGMEM APPSKEY[16] = {0xA5, 0x9B, 0x56, 0x5E, 0xD0, 0xC7, 0x08, 0xCC, 0x2B, 0xF2, 0x7D, 0x8B, 0x71, 0x40, 0xED, 0x90};
//
///* big-endian */ // f358b74f3603036b463ec486dbb22955
//static const PROGMEM u1_t NWKSKEY[16] = {0xF3, 0x58, 0xB7, 0x4F, 0x36, 0x03, 0x03, 0x6B, 0x46, 0x3E, 0xC4, 0x86, 0xDB, 0xB2, 0x29, 0x55};
//
///* big-endian */ // 26031738
//static const u4_t DEVADDR = 0x26031738; // <-- Change this address for every node!

/* 
 * These callbacks are only used in over-the-air activation, so they are
 * left empty here (we cannot leave them out completely unless
 * DISABLE_JOIN is set in config.h, otherwise the linker will complain).
 */
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

/* Data to Send. */
static uint8_t mydata[] = "AdailSilva-IoT";

/* Instances. */
/* Jobs: */
static osjob_t sendjob;

/* Schedule TX every this many seconds (might become longer due to duty cycle limitations). */
const unsigned TX_INTERVAL = 10;

/* Pin mapping. */
const lmic_pinmap lmic_pins = {
  .nss = RADIO_NSS_PORT,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RADIO_RESET_PORT,
  .dio = { RADIO_DIO_0_PORT, RADIO_DIO_1_PORT, LMIC_UNUSED_PIN },  // DIO2 não é utilizado.
};

/*****************************
 _____           _      
/  __ \         | |     
| /  \/ ___   __| | ___ 
| |    / _ \ / _` |/ _ \
| \__/\ (_) | (_| |  __/
 \____/\___/ \__,_|\___|
*****************************/

/* Functions. */
void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    /*
     * This event is defined but not used in the code.
     * No point in wasting codespace on it.
     * case EV_RFU1:
     * Serial.println(F("EV_RFU1"));
     * break;
     */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack;"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" byte(s) of payload."));
      }
      /* Schedule next transmission. */
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      /* Data received in ping slot. */
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
     * This event is defined but not used in the code.
     * No point in wasting codespace on it.
     * case EV_SCAN_FOUND:
     *   Serial.println(F("EV_SCAN_FOUND"));
     *   break;
     */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void do_send(osjob_t *j) {
  /* Check if there is not a current TX/RX job running. */
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    /* Prepare upstream data transmission at the next possible time. */
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
  /* Next TX is scheduled after TX_COMPLETE event. */
}

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/

void setup() {

  SPI.begin(RADIO_SCLK_PORT, RADIO_MISO_PORT, RADIO_MOSI_PORT, RADIO_NSS_PORT);

  Serial.begin(9600);

  delay(100);

  Serial.println(F("Starting..."));

  /* For Pinoccio Scout boards. */
  #ifdef VCC_ENABLE
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif

  /* LMIC init. */
  os_init();

  /* Reset the MAC state. Session and pending data transfers will be discarded. */
  LMIC_reset();

/* 
 * Set static session parameters. Instead of dynamically establishing a session
 * by joining the network, precomputed session parameters are be provided.
 */
#ifdef PROGMEM
  /* 
   * On AVR, these values are stored in flash and only copied to RAM once.
   * Copy them to a temporary buffer here, LMIC_setSession will copy them into a buffer of its own again.
   */
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  // LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
#else
  /* If not running an AVR with PROGMEM, just use the arrays directly. */
  //LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
#endif

  //LMIC_selectSubBand(1);

  /* Channels Control to AU915 (8 at 15 + 65 channels): */
  for (u1_t b = 0; b < 8; ++b) {
    LMIC_disableSubBand(b);
  }

  for (u1_t channel = 0; channel < 72; ++channel) {
    LMIC_disableChannel(channel);
  }

  //LMIC_enableChannel(0);  // 915.2 MHz.
  //LMIC_enableChannel(1);  // 915.4 MHz.
  //LMIC_enableChannel(2);  // 915.6 MHz.
  //LMIC_enableChannel(3);  // 915.8 MHz
  //LMIC_enableChannel(4);  // 916.0 MHz.
  //LMIC_enableChannel(5);  // 916.2 MHz.
  //LMIC_enableChannel(6);  // 916.4 MHz.
  //LMIC_enableChannel(7);  // 916.6 MHz.

  LMIC_enableChannel(8);  // 916.8 MHz.
  LMIC_enableChannel(9);  // 917.0 MHz.
  LMIC_enableChannel(10); // 917.2 MHz.
  LMIC_enableChannel(11); // 917.4 MHz
  LMIC_enableChannel(12); // 917.6 MHz.
  LMIC_enableChannel(13); // 917.8 MHz.
  LMIC_enableChannel(14); // 918.0 MHz.
  LMIC_enableChannel(15); // 918.2 MHz.

  //LMIC_enableChannel(64); /* Test */
  //LMIC_enableChannel(65); /* Test */

  /* Disable Adaptive Data Rate. */
  LMIC_setAdrMode(0);

  /* Disable link check validation. */
  LMIC_setLinkCheckMode(0);

  /* TTN uses SF9 for its RX2 window. */
  LMIC.dn2Dr = DR_SF9;

  /* Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library). */
  LMIC_setDrTxpow(DR_SF7, 14);

  /* 
   * Use with Arduino Pro Mini ATmega328P 3.3V 8MHz;
   * Let LMIC compensate for +/- 1% clock error.
   */
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  /* Start job. */
  do_send(&sendjob);
}

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/

void loop() {
  os_runloop_once();
}
