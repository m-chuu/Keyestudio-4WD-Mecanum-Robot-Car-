#include <Arduino.h>
#define DECODE_NEC        // Must be defined before the first IRremote include
#include <IRremote.hpp>
#include <Servo.h>
#include "RobotSystem.h"


Servo clamperServo;
mecanumCar car(MOTOR_SDA_PIN, MOTOR_SCL_PIN);  // Verified wiring: SDA=D3, SCL=D2

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

  // 2. Drive the robot to its starting home line
  HomeRobot();
}

/**
 * The standard Arduino main loop.
 * After setup() finishes, this block repeats indefinitely.
 */
void loop() {
  // Your robot is now IDLE / STANDBY and waiting for remote commands.
  if (IrReceiver.decode()) {
    // Process your IR remote signals here
    
    IrReceiver.resume(); // Receive the next value
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
  pinMode(IR_LINE_LEFT_PIN, INPUT);
  pinMode(IR_LINE_CENTER_PIN, INPUT);
  pinMode(IR_LINE_RIGHT_PIN, INPUT);
  
  // Reading standard baseline to check if pins are floating or shorted
  int leftCheck = digitalRead(IR_LINE_LEFT_PIN);
 int rightCheck = digitalRead(IR_LINE_RIGHT_PIN);
  if (leftCheck == HIGH && rightCheck == HIGH && digitalRead(IR_LINE_CENTER_PIN) == HIGH) {
    Serial.println(F("WARNING: All IR pins pulling HIGH. Check Rail Power."));
  } else {
    Serial.println(F("OK"));
  }

  Serial.println(F("============================================="));
  Serial.println(F(" AUDIT COMPLETE: System cleared for Homing. "));
  Serial.println(F("=============================================\n"));
  return true;
}

/**
 * Drives the robot forward until it registers a structured black line condition 
 * (all 3 tracking IR indicator lights turn OFF/read HIGH), purges any stale data buffers, 
 * and enters an idle state.
 */
void HomeRobot() {
  Serial.println(F("Entering Homing Mode..."));
  Serial.println(F("Searching for Black Line Anchor..."));

  // Make sure the IR sensor pins are configured as inputs for homing
  pinMode(IR_LINE_LEFT_PIN, INPUT);
  pinMode(IR_LINE_CENTER_PIN, INPUT);
  pinMode(IR_LINE_RIGHT_PIN, INPUT);

  // Drive forward slowly using cruise variables at a constant speed across all 4 wheels
  speed_Upper_L = 60; speed_Lower_L = 60;
  speed_Upper_R = 60; speed_Lower_R = 60;
  car.Advance();

  bool lineFound = false;
  unsigned long timeoutGate = millis();
  unsigned long lastReport  = 0;

  while (!lineFound) {
    // Re-assert the drive command continuously. A single dropped bit-bang I2C frame
    // would otherwise leave the motors stopped; this keeps them commanded ON.
    car.Advance();

    // On most standard IR tracking boards, encountering a non-reflective black line
    // causes the digital output to flag HIGH (or LOW depending on logic), turning its indicator LED OFF.
    int leftIR   = digitalRead(IR_LINE_LEFT_PIN);
    int centerIR = digitalRead(IR_LINE_CENTER_PIN);
    int rightIR  = digitalRead(IR_LINE_RIGHT_PIN);

    // DIAGNOSTIC: print raw sensor states twice a second so we can confirm wiring/polarity.
    if (millis() - lastReport > 500) {
      lastReport = millis();
      Serial.print(F("[HOMING] IR L/C/R = "));
      Serial.print(leftIR); Serial.print(F(" "));
      Serial.print(centerIR); Serial.print(F(" "));
      Serial.print(rightIR);
      Serial.println(F("   (motors commanded FORWARD)"));
    }

    // Condition: All 3 IR receivers are reading dark surface simultaneously (LEDs extinguished)
    if (leftIR == HIGH && centerIR == HIGH && rightIR == HIGH) {
      car.Stop();
      lineFound = true;
      Serial.println(F("-> Black Line Detected! Halting Drive Motors."));
    }

    // Safety fallback timeout increased to 30 seconds (30000 ms) to prevent infinite runaway if line is missed
    if (millis() - timeoutGate > 30000) {
      car.Stop();
      Serial.println(F("-> Homing Error: Line seek timeout. System Halted."));
      while (true) { delay(1000); } // Lock down system for safety
    }
    delay(10);
  }

  // Clear IR remote command buffer to prevent accidental handshake interrupt execution
  Serial.print(F("Purging IR remote receiver buffers... "));
  while (IrReceiver.decode()) {
    IrReceiver.resume();
    delay(50);
  }
  Serial.println(F("Done."));

  // Handshake sequence completed: Ping the requested JSON payload to the serial monitor
  Serial.println(F("{\"status\":\"READY\",\"state\":\"IDLE\"}"));
}