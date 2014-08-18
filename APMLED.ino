#include "Adafruit_TLC59711.h"
#include <SPI.h>
#include <EEPROM.h>
#include "../mavlink/include/mavlink.h"        // Mavlink interface
#include "../mavlink/include/pixhawk/mavlink.h"        // Mavlink interface

#define data   2
#define clock  6
#define NUM_TLC59711 1

#define INTERVAL_TIME   50

#define MODE_OFF        0
#define MODE_SOLID      1
#define MODE_BLINK_SLOW 2
#define MODE_BLINK_FAST 3
#define MODE_FLASH_REAR 4
#define MODE_FLASH_ALL  5
#define MODE_FLASH_ALL_FASTER  6

#define PATTERN_LENGTH  32

typedef struct s_ledMode {
  uint8_t led;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint32_t pattern;
  uint8_t patternLength;
  uint8_t patternPosition;
  uint16_t lastValue;
} t_ledMode;

uint16_t ledLevelOff = 0;
uint16_t ledLevelOn = 65535;
float ledDecayFactor = 1.5f;

uint8_t ledMode = 5;
uint8_t lastLedMode = 0xff;
uint8_t patternPosition = 0;

uint32_t nextCycle = 0;

t_ledMode allLeds[4] = {
  {0, 1,1,1, 0,0,0, 0}, 
  {1, 1,1,1, 0,0,0, 0}, 
  {2, 1,1,1, 0,0,0, 0}, 
  {3, 1,1,1, 0,0,0, 0}
};

mavlink_heartbeat_t heartbeat;
mavlink_message_t msg;
mavlink_status_t status;

Adafruit_TLC59711 tlc = Adafruit_TLC59711(NUM_TLC59711, clock, data);

uint8_t statusLedState = 1;

void setup() {
  Serial.begin(57600);
  Serial2.begin(57600);
  
  ledMode = MODE_OFF;
  pinMode(2, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);
  tlc.begin();
  tlc.write();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '5') {
      ledMode = c - '0';
      EEPROM.write(0, ledMode);
    }
  }

  int read = 0;
  while(Serial2.available()) {
    // TODO: only read up to (BAUDRATE / Hz of loop or so)
    uint8_t c = Serial2.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
//      Serial.print("MAV, msgid:");
//      Serial.println(msg.msgid);
      /*
      stab      81  / 0               209 / 0
      alt hold  81  / 2   0.0010      
      pos hold  89  / 16  1.0000
      rtl       89  / 6   0.0110
      mission   89  / 3   0.0011
      */
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          mavlink_msg_heartbeat_decode(&msg, &heartbeat);
          Serial.print("\tbase_mode:\t");
          Serial.println(heartbeat.base_mode);
          Serial.print("\tcustom_mode:\t");
          Serial.println(heartbeat.custom_mode);
          if (!(heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)) {
            // unarmed
            ledLevelOn = 4096;
            ledMode = MODE_BLINK_SLOW;
          } else {
            // armed
            ledLevelOn = 65535;

            switch(heartbeat.system_status) {
              // blink fast in case of emergency
              case MAV_STATE_CRITICAL:
              case MAV_STATE_EMERGENCY:
                ledMode = MODE_BLINK_FAST;
                break;

              default:
                // don't flash front unless autonomous
              if (heartbeat.base_mode & MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE) {
                switch(heartbeat.custom_mode) {
                  case 0: // stabelize
                  case 2: // alt hold
                  case 16: // pos hold
                  case 11: //drift
                    ledMode = MODE_FLASH_REAR;
                    break;
                  case 3:
                    ledMode = MODE_FLASH_ALL;
                    break;
                  case 6:
                    ledMode = MODE_FLASH_ALL_FASTER;
                    break;
                  default:
                    ledMode = MODE_BLINK_SLOW;
                    break;
                }
              }
            }
          }
          statusLedState = 1 - statusLedState;
          digitalWrite(13, statusLedState);
          break;
      }
    }
    if (read++ > 64)
      break;
  }

  if (ledMode != lastLedMode) {
    uint8_t i;
    ledDecayFactor = 1.5f;
    for(i=0;i<4;i++) {
      allLeds[i].patternPosition = 0;
    }
    switch(ledMode) {
      case MODE_OFF:
        for(i=0;i<4;i++) {
          allLeds[i].pattern = 0;
          allLeds[i].patternLength = 32;
        }
        break;

      case MODE_SOLID:
        for(i=0;i<4;i++) {
          allLeds[i].pattern = 0b1;
          allLeds[i].patternLength = 1;
        }
        break;

      case MODE_BLINK_SLOW:
        ledDecayFactor = 1.5f;
        for(i=0;i<4;i++) {
          allLeds[i].pattern = 0b111111111111000000000000;
          allLeds[i].patternLength = 24;
        }
        break;

      case MODE_BLINK_FAST:
        ledDecayFactor = 4;
        for(i=0;i<4;i++) {
          allLeds[i].pattern = 0b1111000011110000;
          allLeds[i].patternLength = 16;
        }
        break;

      case MODE_FLASH_REAR:
          ledDecayFactor = 65535;
          allLeds[0].pattern  = 0b1111111111111111;
          allLeds[0].patternLength = 16;
          allLeds[1].pattern = 0b1111111111111111;
          allLeds[1].patternLength = 16;
          allLeds[2].pattern   = 0b1010000000000000;
          allLeds[2].patternLength = 32;
          allLeds[3].pattern  = 0b0000101000000000;
          allLeds[3].patternLength = 31;
        break;

      case MODE_FLASH_ALL:
          ledDecayFactor = 65535;
          allLeds[0].pattern = 0b101;
          allLeds[1].pattern = 0b101;
          allLeds[2].pattern = 0b101;
          allLeds[3].pattern = 0b101;
          allLeds[0].patternLength = 32;
          allLeds[1].patternLength = 30;
          allLeds[2].patternLength = 31;
          allLeds[3].patternLength = 29;
        break;

        case MODE_FLASH_ALL_FASTER:
          ledDecayFactor = 65535;
          allLeds[0].pattern = 0b101;
          allLeds[1].pattern = 0b101;
          allLeds[2].pattern = 0b101;
          allLeds[3].pattern = 0b101;
          allLeds[0].patternLength = 13;
          allLeds[1].patternLength = 16;
          allLeds[2].patternLength = 16;
          allLeds[3].patternLength = 13;
          break;
    }
    lastLedMode = ledMode;
  }

  uint32_t now = millis();
  if (nextCycle <= now) {
    for(int i=0;i<4;i++) {
      uint16_t ledState = ((allLeds[i].pattern >> (allLeds[i].patternLength - 1 - allLeds[i].patternPosition)) & 1);
      uint16_t nextValue;
      if (ledState) {
        nextValue = ledLevelOn;
      } else {
        nextValue = (uint16_t)((float)allLeds[i].lastValue / ledDecayFactor);
        nextValue = (nextValue<ledLevelOff?ledLevelOff:nextValue);
      }

      tlc.setLED(i, allLeds[i].r * nextValue, allLeds[i].g * nextValue, allLeds[i].b * nextValue);
      allLeds[i].patternPosition = (allLeds[i].patternPosition + 1) % allLeds[i].patternLength;
      allLeds[i].lastValue = nextValue;
    }
    tlc.write();
    nextCycle = now + INTERVAL_TIME;
  }
}

