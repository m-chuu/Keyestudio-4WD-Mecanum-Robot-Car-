/*
  ================================================================
  MECANUM TURN CALIBRATION SKETCH
  Dynamic 90° and 180° Turn Timing Calibration via IR Remote
  ================================================================
  IR Controls:
  - Up Button (0x46):     Increase delay90 & delay180 by 25ms
  - Down Button (0x15):   Decrease delay90 & delay180 by 25ms
  - Left Button (0x44):   Execute 90° Left turn
  - Right Button (0x43):  Execute 90° Right turn
  - OK Button (0x40):     Execute 180° Right turn

  Calibrated values automatically saved to EEPROM.
  ================================================================
*/

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <EEPROM.h>
#define DECODE_NEC
#include <IRremote.hpp>

// ── IR PIN ────────────────────────────────────────────────────
#define RECV_PIN        A3

// ── IR COMMAND CODES (Standard NEC) ───────────────────────────
#define CMD_UP          0x46    // Increase delay
#define CMD_DOWN        0x15    // Decrease delay
#define CMD_LEFT        0x44    // 90° Left turn
#define CMD_RIGHT       0x43    // 90° Right turn
#define CMD_OK          0x40    // 180° Right turn

// ── MOTOR CONFIGURATION ───────────────────────────────────────
#define LINE_SPEED_L    80
#define LINE_SPEED_R    68

// ── EEPROM ADDRESSES ──────────────────────────────────────────
#define EEPROM_ADDR_DELAY90   0    // int (2 bytes)
#define EEPROM_ADDR_DELAY180  2    // int (2 bytes)

// ── CALIBRATION GLOBALS ───────────────────────────────────────
int delay90 = 500;      // Starting value for 90° turn (ms)
int delay180 = 1000;    // Starting value for 180° turn (ms)
const int STEP_SIZE = 25;
const int MIN_DELAY = 100;
const int MAX_DELAY = 2000;

// ── MOTOR CONTROLLER ──────────────────────────────────────────
mecanumCar car(3, 2);

// ================================================================
//  EEPROM MANAGEMENT
// ================================================================
void loadCalibrationFromEEPROM() {
  EEPROM.get(EEPROM_ADDR_DELAY90, delay90);
  EEPROM.get(EEPROM_ADDR_DELAY180, delay180);

  // Validate ranges (in case EEPROM is uninitialized)
  if (delay90 < MIN_DELAY || delay90 > MAX_DELAY) {
    delay90 = 500;
  }
  if (delay180 < MIN_DELAY || delay180 > MAX_DELAY) {
    delay180 = 1000;
  }

  Serial.println(F("\n[STARTUP] Calibration values loaded from EEPROM:"));
  Serial.print(F("[CALIB] delay90 = "));
  Serial.print(delay90);
  Serial.print(F("ms, delay180 = "));
  Serial.print(delay180);
  Serial.println(F("ms"));
}

void saveCalibrationToEEPROM() {
  EEPROM.put(EEPROM_ADDR_DELAY90, delay90);
  EEPROM.put(EEPROM_ADDR_DELAY180, delay180);
  Serial.println(F("[EEPROM] Calibration saved!"));
}

// ================================================================
//  TURN FUNCTIONS
// ================================================================
void Turn_Left(int duration_ms) {
  // Counter-clockwise: left wheels backward, right wheels forward
  car.Motor_Upper_L(0, LINE_SPEED_L);  // Left backward
  car.Motor_Lower_L(0, LINE_SPEED_L);  // Left backward
  car.Motor_Upper_R(1, LINE_SPEED_R);  // Right forward
  car.Motor_Lower_R(1, LINE_SPEED_R);  // Right forward

  delay(duration_ms);
  car.Stop();
}

void Turn_Right(int duration_ms) {
  // Clockwise: left wheels forward, right wheels backward
  car.Motor_Upper_L(1, LINE_SPEED_L);  // Left forward
  car.Motor_Lower_L(1, LINE_SPEED_L);  // Left forward
  car.Motor_Upper_R(0, LINE_SPEED_R);  // Right backward
  car.Motor_Lower_R(0, LINE_SPEED_R);  // Right backward

  delay(duration_ms);
  car.Stop();
}

// ================================================================
//  IR BUTTON HANDLERS
// ================================================================
void increaseDelay() {
  delay90 = min(delay90 + STEP_SIZE, MAX_DELAY);
  delay180 = min(delay180 + STEP_SIZE, MAX_DELAY);
  Serial.print(F("[CALIB] delay90 = "));
  Serial.print(delay90);
  Serial.print(F("ms, delay180 = "));
  Serial.print(delay180);
  Serial.println(F("ms"));
  saveCalibrationToEEPROM();
}

void decreaseDelay() {
  delay90 = max(delay90 - STEP_SIZE, MIN_DELAY);
  delay180 = max(delay180 - STEP_SIZE, MIN_DELAY);
  Serial.print(F("[CALIB] delay90 = "));
  Serial.print(delay90);
  Serial.print(F("ms, delay180 = "));
  Serial.print(delay180);
  Serial.println(F("ms"));
  saveCalibrationToEEPROM();
}

void test90Left() {
  Serial.print(F("[TEST] Executing 90° LEFT turn ("));
  Serial.print(delay90);
  Serial.println(F("ms)"));
  Turn_Left(delay90);
  Serial.println(F("[TEST] 90° LEFT complete. Press a button to continue."));
}

void test90Right() {
  Serial.print(F("[TEST] Executing 90° RIGHT turn ("));
  Serial.print(delay90);
  Serial.println(F("ms)"));
  Turn_Right(delay90);
  Serial.println(F("[TEST] 90° RIGHT complete. Press a button to continue."));
}

void test180Right() {
  Serial.print(F("[TEST] Executing 180° RIGHT turn ("));
  Serial.print(delay180);
  Serial.println(F("ms)"));
  Turn_Right(delay180);
  Serial.println(F("[TEST] 180° RIGHT complete. Press a button to continue."));
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }

  Serial.println(F("\n============================================="));
  Serial.println(F("   MECANUM TURN CALIBRATION SKETCH"));
  Serial.println(F("============================================="));

  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  loadCalibrationFromEEPROM();

  Serial.println(F("\n─── INSTRUCTIONS ───────────────────────────"));
  Serial.println(F("UP (0x46):    Increase delay by 25ms"));
  Serial.println(F("DOWN (0x15):  Decrease delay by 25ms"));
  Serial.println(F("LEFT (0x44):  Test 90° Left turn"));
  Serial.println(F("RIGHT (0x43): Test 90° Right turn"));
  Serial.println(F("OK (0x40):    Test 180° Right turn"));
  Serial.println(F("─────────────────────────────────────────────\n"));
  Serial.println(F("Ready for calibration. Press a button."));
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  if (IrReceiver.decode()) {
    uint8_t key = IrReceiver.decodedIRData.command;
    IrReceiver.resume();

    switch (key) {
      case CMD_UP:
        increaseDelay();
        break;

      case CMD_DOWN:
        decreaseDelay();
        break;

      case CMD_LEFT:
        test90Left();
        break;

      case CMD_RIGHT:
        test90Right();
        break;

      case CMD_OK:
        test180Right();
        break;

      default:
        Serial.print(F("[DEBUG] Unknown IR code: 0x"));
        Serial.println(key, HEX);
        break;
    }
  }

  delay(50);
}
