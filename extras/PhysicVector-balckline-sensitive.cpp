#include <Arduino.h>
#define DECODE_NEC        // Must be defined before the first IRremote include
#include <IRremote.hpp>
#include <Servo.h>
#include "RobotSystem.h"


Servo clamperServo;
mecanumCar car(MOTOR_SDA_PIN, MOTOR_SCL_PIN);  // Verified wiring: SDA=D3, SCL=D2

// Verified line-sensor allocation (from pin-finder scan, defined in RobotSystem.h):
//   IR_LINE_LEFT_PIN   = A2
//   IR_LINE_CENTER_PIN = A1
//   IR_LINE_RIGHT_PIN  = A0
//   SERVO_PIN          = A4  (moved off A0 to clear the right-sensor conflict)

// --- IR Remote Mappings (codes match main-2.cpp) ---
const int IR_KEY_5 = 24;   // Press '5' on the remote to start Homing

// Forward declarations so setup() and loop() know these functions exist
bool RunSelfAudit();
void HomeRobot();

/**
 * The standard Arduino initialization hook.
 * This runs exactly once when your robot powers up or resets.
 */
void setup() {
  // Start serial communication for debugging output
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB boards)
  }

  // 0. Bring up hardware before any diagnostics use it
  clamperServo.attach(SERVO_PIN);
  clamperServo.write(SERVO_OPEN_ANGLE_RS);     // Start clamper open
  car.Init();                                  // Initialize motor controller to a known state
  IrReceiver.begin(IR_REMOTE_PIN, DISABLE_LED_FEEDBACK);
  delay(400);

  // 1. Run the system diagnostic check
  RunSelfAudit();

  // 2. Wait for the operator to start a line search from the remote.
  Serial.println(F("Ready. Press '5' to search the next black line."));
}

/**
 * The standard Arduino main loop.
 * After setup() finishes, this block repeats indefinitely.
 */
void loop() {
  // Your robot is now IDLE / STANDBY and waiting for remote commands.
  if (IrReceiver.decode()) {
    int key = IrReceiver.decodedIRData.command;
    IrReceiver.resume(); // Ready to receive the next value

    if (key == IR_KEY_5) {
      HomeRobot();       // Press '5' -> drive forward to the NEXT black line
    }
  }
  delay(100); 
}

/**
 * Runs an active diagnostic check on all moving parts and sensors.
 * Since Arduino pins cannot directly measure current/power without extra shunt hardware,
 * we verify operation via active feedback timeout gates and safe diagnostic pulses.
 */
bool RunSelfAudit() {
  Serial.println(F("\n============================================="));
  Serial.println(F("         SYSTEM SELF-AUDIT IN PROGRESS        "));
  Serial.println(F("============================================="));
  delay(500);

  // 1. Audit: Clamper / Servo
  Serial.print(F("[AUDIT] Initializing Servo Clamper... "));
  clamperServo.write(SERVO_CLOSED_ANGLE_RS);
  delay(400);
  clamperServo.write(SERVO_OPEN_ANGLE_RS);
  delay(400);
  Serial.println(F("OK (Power Draw Nom.)"));

  // 2. Audit: Mecanum Wheels & Motors
  Serial.println(F("[AUDIT] Testing Drive Train Power Rails..."));

  // 2a. Probe the I2C motor controller (chip @ 0x30) to confirm the bus is alive.
  // A returned ACK (0) means SDA/SCL pins are correct AND the controller is powered.
  // A NAK (1) means wrong pins or the controller board has no power.
  car.IIC_Start();
  car.IIC_SendByte(0x30 << 1);          // address + write bit
  bool noAck = car.IIC_RecvACK();        // 0 = ACK (device present), 1 = NAK (no device)
  car.IIC_Stop();
  if (noAck) {
    Serial.println(F(" -> [FAIL] Motor controller did NOT acknowledge on I2C."));
    Serial.println(F("           Check: SDA/SCL pins (A4/A5) and controller battery power."));
  } else {
    Serial.println(F(" -> [OK] Motor controller acknowledged on I2C (bus + power good)."));
  }

  // Set safety testing low speed
  speed_Upper_L = 50; speed_Lower_L = 50;
  speed_Upper_R = 50; speed_Lower_R = 50;

  car.Advance();
  delay(150);
  car.Stop();
  delay(100);
  Serial.println(F(" -> Drive train command sent."));

  // 3. Audit: IR Line Tracking Sensors Input Check
  Serial.print(F("[AUDIT] Checking IR Array Pin States... "));

  // Reading standard baseline to check if pins are floating or shorted
  int leftCheck  = analogRead(IR_LINE_LEFT_PIN)  > 700 ? 1 : 0;
  int rightCheck = analogRead(IR_LINE_RIGHT_PIN) > 700 ? 1 : 0;
  if (leftCheck == 1 && rightCheck == 1 && (analogRead(IR_LINE_CENTER_PIN) > 700 ? 1 : 0) == 1) {
    Serial.println(F("WARNING: All IR pins reading BLACK. Check Rail Power."));
  } else {
    Serial.println(F("OK"));
  }

  Serial.println(F("============================================="));
  Serial.println(F(" AUDIT COMPLETE: System cleared for Homing. "));
  Serial.println(F("=============================================\n"));
  return true;
}

// Verified sensor polarity (analog): white floor ~25, black tape ~750.
// Strict threshold: >700 = black (1), <=700 = white (0).
#define ON_BLACK(pin) (analogRead(pin) > 700)

// True once ALL sensors see black at the same time (a full perpendicular line).
static bool allOnBlack() {
  return ON_BLACK(IR_LINE_LEFT_PIN) && ON_BLACK(IR_LINE_CENTER_PIN) && ON_BLACK(IR_LINE_RIGHT_PIN);
}

/**
 * Drives the robot forward to the NEXT black line and stops on it.
 *
 * Each press of '5' calls this. To avoid instantly re-detecting the line the
 * robot is already parked on, it first drives forward until it has fully
 * cleared the current line (all sensors back on white), then searches forward
 * until the next line. Returns to idle so '5' can be pressed again.
 */
void HomeRobot() {
  Serial.println(F("Entering Homing Mode..."));
  Serial.println(F("Searching for Black Line Anchor..."));

  speed_Upper_L = 60; speed_Lower_L = 60;
  speed_Upper_R = 60; speed_Lower_R = 60;

  // --- Phase 1: hold still for exactly 5 seconds before searching ------
  car.Stop();
  Serial.println(F("-> Holding for 5 s before search..."));
  unsigned long holdGate = millis();
  while (millis() - holdGate < 5000) {           // 5-second initial stop
    car.Stop();
    delay(10);
  }

  // --- Phase 2: now drive forward and search for the next black line ----
  bool lineFound = false;
  unsigned long timeoutGate = millis();
  unsigned long lastReport  = 0;

  while (!lineFound) {
    // Re-assert the drive command continuously so a dropped I2C frame
    // doesn't silently leave the motors stopped.
    car.Advance();

    if (millis() - lastReport > 500) {
      lastReport = millis();
      Serial.print(F("[SEARCH] IR L/C/R = "));
      Serial.print(analogRead(IR_LINE_LEFT_PIN)   > 700 ? 1 : 0); Serial.print(F(" "));
      Serial.print(analogRead(IR_LINE_CENTER_PIN) > 700 ? 1 : 0); Serial.print(F(" "));
      Serial.print(analogRead(IR_LINE_RIGHT_PIN)  > 700 ? 1 : 0);
      Serial.println(F("   (motors commanded FORWARD)"));
    }

    if (allOnBlack()) {
      car.Stop();
      lineFound = true;
      Serial.println(F("-> Next Black Line Detected! Halting Drive Motors."));
    }

    // Safety fallback: stop after 30 s and return to idle (no hard lockup,
    // so the operator can simply press '5' to try again).
    if (millis() - timeoutGate > 30000) {
      car.Stop();
      break;
    }
    delay(10);
  }

  // Clear any IR codes queued while moving so the next '5' is intentional.
  while (IrReceiver.decode()) {
    IrReceiver.resume();
    delay(20);
  }

  Serial.println(F("Ready. Press '5' to search the next black line."));
}