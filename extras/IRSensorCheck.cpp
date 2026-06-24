#include <Arduino.h>

// =====================================================================
//  IRSensorCheck.cpp
//  Checks the 3 line-tracking IR sensors SEPARATELY on the Serial Monitor.
//
//  VERIFIED WIRING (found via pin-finder scan):
//      RIGHT  = A0
//      CENTER = A1
//      LEFT   = A2
//  (RobotSystem.h's old A4/A5/A2 guess was wrong -> floating reads.)
//
//  VERIFIED POLARITY (raw analogRead):
//      white / reflective floor  ~=  25   (sensor LED lit)
//      black tape / line         ~= 700..820 (sensor LED off)
//  => "line detected" when raw is ABOVE the threshold.
//
//  Output: 1 = black line detected, 0 = no line.
// =====================================================================

// --- Line sensor pins (per main-2.cpp these were free; A3=IR remote) ---
const uint8_t IR_LINE_RIGHT_PIN  = A0;
const uint8_t IR_LINE_CENTER_PIN = A1;
const uint8_t IR_LINE_LEFT_PIN   = A2;

// --- Detection tuning -------------------------------------------------
// white ~25, black ~700+, so 400 sits safely between the two.
const int  THRESHOLD         = 400;
const bool DETECT_WHEN_ABOVE = true;  // black line = HIGH raw => detected

const unsigned long REPORT_MS = 250;
unsigned long lastReport = 0;

int detected(int raw) {
  bool above = (raw > THRESHOLD);
  return (DETECT_WHEN_ABOVE ? above : !above) ? 1 : 0;
}

void printSensor(const __FlashStringHelper *name, int raw) {
  Serial.print(name);
  Serial.print(F(" raw="));
  Serial.print(raw);
  Serial.print(F(" ["));
  Serial.print(detected(raw) ? F("DETECT") : F("------"));
  Serial.print(F("]   "));
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  Serial.println(F("====================================="));
  Serial.println(F("   3x IR LINE SENSOR CHECK (calib.)  "));
  Serial.println(F("====================================="));
  Serial.println(F("Pins -> LEFT=A2  CENTER=A1  RIGHT=A0"));
  Serial.println(F("1 = black line detected, 0 = no line.\n"));
}

void loop() {
  if (millis() - lastReport < REPORT_MS) return;
  lastReport = millis();

  int rawLeft   = analogRead(IR_LINE_LEFT_PIN);
  int rawCenter = analogRead(IR_LINE_CENTER_PIN);
  int rawRight  = analogRead(IR_LINE_RIGHT_PIN);

  printSensor(F("LEFT"),   rawLeft);
  printSensor(F("CENTER"), rawCenter);
  printSensor(F("RIGHT"),  rawRight);

  Serial.print(F("L/C/R = "));
  Serial.print(detected(rawLeft));   Serial.print(' ');
  Serial.print(detected(rawCenter)); Serial.print(' ');
  Serial.println(detected(rawRight));
}
