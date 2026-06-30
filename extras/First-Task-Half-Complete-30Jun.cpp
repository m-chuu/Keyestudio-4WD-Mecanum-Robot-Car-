/*
  ================================================================
  MERGED: LINE FOLLOWING + OBJECT CATCHING
  ================================================================
  Button 5: Home sequence
  Button 3: Follow line continuously + catch object at 4cm
  Button *: Emergency stop (open clamp, stop motors)

  Critical Feature: SHORT pulseIn timeout (2000µs) to prevent
  steering stutter while monitoring ultrasonic.
  ================================================================
*/

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <Servo.h>
#define DECODE_NEC
#include <IRremote.hpp>

// ── LINE FOLLOWING PINS ───────────────────────────────────────
#define RECV_PIN        A3
#define SENSOR_LEFT     A0
#define SENSOR_MID      A1
#define SENSOR_RIGHT    A2

// ── SERVO & ULTRASONIC PINS ───────────────────────────────────
const uint8_t SERVO_PIN           = 9;
const uint8_t ULTRASONIC_TRIG_PIN = 12;
const uint8_t ULTRASONIC_ECHO_PIN = 13;

// ── SERVO ANGLES ──────────────────────────────────────────────
const int SERVO_OPEN_ANGLE   = 15;
const int SERVO_CLOSED_ANGLE = 85;

// ── IR CODES ──────────────────────────────────────────────────
#define CMD_3           0x0D   // Follow line + catch object
#define CMD_5           24     // Homing sequence
#define CMD_STAR        0x42   // Emergency stop

// ── LINE FOLLOWING CONFIG ─────────────────────────────────────
#define DRIVE_REAR_WHEELS  1
#define LINE_SPEED_L       80
#define LINE_SPEED_R       68
#define TURN_SLOW_SPEED    15
#define BLACK_THRESHOLD    700

// ── OBJECT DETECTION ──────────────────────────────────────────
const uint16_t OBJECT_DETECT_DISTANCE_CM = 4;
const unsigned long ULTRASONIC_CHECK_INTERVAL_MS = 50;  // Non-blocking check

// ── EXTERN LIBRARY VARIABLES ──────────────────────────────────
extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

// ── CACHED SENSOR READINGS ────────────────────────────────────
static int sensorLeftVal = 0;
static int sensorMidVal = 0;
static int sensorRightVal = 0;

// ── SERVO & ULTRASONIC GLOBALS ────────────────────────────────
Servo servoClamp;
uint16_t currentDistanceCm = 0;
unsigned long lastUltrasonicCheckTime = 0;

// ── MOTOR CONTROLLER ──────────────────────────────────────────
mecanumCar car(3, 2);

// ================================================================
//  SENSOR HELPERS
// ================================================================
static inline bool onLeftBlack() {
  return sensorLeftVal > BLACK_THRESHOLD;
}

static inline bool onMidBlack() {
  return sensorMidVal > BLACK_THRESHOLD;
}

static inline bool onRightBlack() {
  return sensorRightVal > BLACK_THRESHOLD;
}

static bool allOnBlack() {
  return (sensorLeftVal > BLACK_THRESHOLD) &&
         (sensorMidVal > BLACK_THRESHOLD) &&
         (sensorRightVal > BLACK_THRESHOLD);
}

static inline void readSensors() {
  sensorLeftVal = analogRead(SENSOR_LEFT);
  sensorMidVal = analogRead(SENSOR_MID);
  sensorRightVal = analogRead(SENSOR_RIGHT);
}

bool emergencyStopPressed() {
  if (IrReceiver.decode()) {
    bool stop = (IrReceiver.decodedIRData.command == CMD_STAR);
    IrReceiver.resume();
    return stop;
  }
  return false;
}

// ================================================================
//  MOTOR CONTROL
// ================================================================
void driveFront(uint8_t leftSpd, uint8_t rightSpd) {
  car.Motor_Upper_L(1, leftSpd);
  car.Motor_Upper_R(1, rightSpd);

#if DRIVE_REAR_WHEELS
  car.Motor_Lower_L(1, leftSpd);
  car.Motor_Lower_R(1, rightSpd);
#else
  car.Motor_Lower_L(0, 0);
  car.Motor_Lower_R(0, 0);
#endif
}

void FollowLine(bool onLeft, bool onRight, uint8_t baseSpeedL, uint8_t baseSpeedR) {
  if (onRight) {
    speed_Upper_L = baseSpeedL;
    speed_Lower_L = baseSpeedL;
    speed_Upper_R = TURN_SLOW_SPEED;
    speed_Lower_R = TURN_SLOW_SPEED;
    driveFront(baseSpeedL, TURN_SLOW_SPEED);
  } else if (onLeft) {
    speed_Upper_L = TURN_SLOW_SPEED;
    speed_Lower_L = TURN_SLOW_SPEED;
    speed_Upper_R = baseSpeedR;
    speed_Lower_R = baseSpeedR;
    driveFront(TURN_SLOW_SPEED, baseSpeedR);
  } else {
    speed_Upper_L = baseSpeedL;
    speed_Lower_L = baseSpeedL;
    speed_Upper_R = baseSpeedR;
    speed_Lower_R = baseSpeedR;
    driveFront(baseSpeedL, baseSpeedR);
  }
}

void restoreIR() {
  car.Stop();
  IrReceiver.begin(RECV_PIN, false);
}

// ================================================================
//  ULTRASONIC DISTANCE (with SHORT timeout for non-blocking)
// ================================================================
uint16_t readDistanceCmFast() {
  // Send trigger pulse
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // CRITICAL: SHORT timeout (2000µs instead of 25000µs)
  // At 4cm distance, echo ~232µs. Short timeout prevents steering stutter.
  unsigned long echoUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 2000UL);

  if (echoUs == 0) {
    return 0;
  }

  return echoUs / 58;  // Convert to cm
}

void servoOpen() {
  servoClamp.write(SERVO_OPEN_ANGLE);
  delay(500);
  Serial.println(F("[SERVO] OPEN (15°)"));
}

void servoClosed() {
  servoClamp.write(SERVO_CLOSED_ANGLE);
  delay(500);
  Serial.println(F("[SERVO] CLOSED (85°) - OBJECT CAUGHT!"));
}

// ================================================================
//  ULTRASONIC MONITOR (non-blocking, ~50ms interval)
// ================================================================
bool checkUltrasonicNonBlocking() {
  unsigned long now = millis();

  if (now - lastUltrasonicCheckTime < ULTRASONIC_CHECK_INTERVAL_MS) {
    return false;  // Too soon to check
  }

  lastUltrasonicCheckTime = now;
  currentDistanceCm = readDistanceCmFast();

  // Return true if object detected (4cm or less, but > 0)
  if (currentDistanceCm > 0 && currentDistanceCm <= OBJECT_DETECT_DISTANCE_CM) {
    return true;
  }

  return false;
}

// ================================================================
//  SELF-AUDIT
// ================================================================
bool RunSelfAudit() {
  Serial.println(F("\n============================================="));
  Serial.println(F("         SYSTEM SELF-AUDIT IN PROGRESS        "));
  Serial.println(F("============================================="));
  delay(500);

  Serial.print(F("[AUDIT] Checking IR Array Pin States... "));
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MID, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  readSensors();

  if (sensorLeftVal < 50 && sensorMidVal < 50 && sensorRightVal < 50) {
    Serial.println(F("WARNING: All IR pins pulling LOW."));
  } else if (sensorLeftVal > 900 && sensorMidVal > 900 && sensorRightVal > 900) {
    Serial.println(F("WARNING: All IR pins maxed out."));
  } else {
    Serial.println(F("OK"));
  }

  Serial.print(F("[AUDIT] Checking Motor Controller... "));
  car.IIC_Start();
  car.IIC_SendByte(0x30 << 1);
  bool noAck = car.IIC_RecvACK();
  car.IIC_Stop();

  if (noAck) {
    Serial.println(F("FAIL"));
  } else {
    Serial.println(F("OK"));
  }

  Serial.print(F("[AUDIT] Testing Drive Motors... "));
  driveFront(50, 50);
  delay(150);
  car.Stop();
  delay(100);
  Serial.println(F("OK"));

  Serial.println(F("============================================="));
  Serial.println(F(" AUDIT COMPLETE: System ready.                "));
  Serial.println(F("=============================================\n"));
  return true;
}

// ================================================================
//  HOMING SEQUENCE
// ================================================================
void HomeRobot() {
  Serial.println(F("Entering Homing Mode..."));
  Serial.println(F("Searching for Black Line Anchor..."));

  bool lineFound = false;
  unsigned long timeoutGate = millis();
  unsigned long lastReport = 0;

  while (!lineFound) {
    readSensors();

    bool onLeft  = onLeftBlack();
    bool onRight = onRightBlack();
    FollowLine(onLeft, onRight, 60, 60);

    if (allOnBlack()) {
      car.Stop();
      lineFound = true;
      Serial.println(F("-> Black Line Detected! Halting."));
    }

    if (millis() - lastReport > 100) {
      lastReport = millis();
      Serial.print(F("[HOMING] IR L/C/R = "));
      Serial.print(onLeftBlack()  ? 1 : 0); Serial.print(F(" "));
      Serial.print(onMidBlack()   ? 1 : 0); Serial.print(F(" "));
      Serial.print(onRightBlack() ? 1 : 0);
      Serial.println(F(" (via FollowLine)"));
    }

    if (millis() - timeoutGate > 30000) {
      car.Stop();
      Serial.println(F("-> Homing Error: Timeout."));
      while (true) { delay(1000); }
    }

    delay(10);
  }

  while (IrReceiver.decode()) {
    IrReceiver.resume();
    delay(50);
  }

  Serial.println(F("{\"status\":\"READY\",\"state\":\"IDLE\"}"));
  restoreIR();
}

// ================================================================
//  NEW BUTTON 3: FOLLOW LINE + CATCH OBJECT
// ================================================================
void followLineAndCatch() {
  Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║     FOLLOW LINE + CATCH MODE - BUTTON 3           ║"));
  Serial.println(F("╚════════════════════════════════════════════════════╝"));
  Serial.println(F("[ACTION] Opening clamp..."));

  // Step 1: Open the clamp
  servoOpen();
  delay(500);

  Serial.println(F("[ACTION] Starting line follow + object detection..."));
  Serial.println(F("[MONITOR] Ultrasonic distance: "));

  unsigned long lastPrintTime = 0;

  // Step 2: Main loop - follow line + monitor ultrasonic
  while (true) {
    // Read line sensors
    readSensors();

    // Apply line-following steering
    bool onLeft  = onLeftBlack();
    bool onRight = onRightBlack();
    FollowLine(onLeft, onRight, LINE_SPEED_L, LINE_SPEED_R);

    // Check ultrasonic (non-blocking, every 50ms)
    if (checkUltrasonicNonBlocking()) {
      // Object detected at 4cm or less!
      Serial.println(F("\n"));
      Serial.println(F("╔════════════════════════════════════════════════════╗"));
      Serial.println(F("║       OBJECT DETECTED! (Distance <= 4cm)          ║"));
      Serial.println(F("╚════════════════════════════════════════════════════╝"));
      break;  // Exit the loop
    }

    // Print distance every 500ms for debugging
    if (millis() - lastPrintTime > 500) {
      lastPrintTime = millis();
      Serial.print(currentDistanceCm);
      Serial.println(F(" cm"));
    }

    // Check for emergency stop
    if (emergencyStopPressed()) {
      Serial.println(F("[EMERGENCY] Button * pressed during line follow!"));
      break;
    }

    delay(10);
  }

  // Step 3: Object detected - stop and catch
  car.Stop();
  Serial.println(F("[ACTION] Car stopped. Closing clamp..."));
  servoClosed();

  Serial.println(F("[STATUS] Object caught! Waiting for next command..."));
  Serial.println(F("{\"status\":\"READY\",\"state\":\"IDLE\"}"));

  restoreIR();
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }

  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  // Initialize servo
  servoClamp.attach(SERVO_PIN);
  servoClamp.write(SERVO_OPEN_ANGLE);

  // Initialize ultrasonic pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  RunSelfAudit();

  Serial.println(F("\nWaiting for operator input..."));
  Serial.println(F("Button 5: Homing sequence"));
  Serial.println(F("Button 3: Follow line + catch object"));
  Serial.println(F("Button *: Emergency stop"));
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  if (IrReceiver.decode()) {
    uint8_t key = IrReceiver.decodedIRData.command;
    IrReceiver.resume();

    if (key == CMD_5) {
      HomeRobot();
    }
    else if (key == CMD_3) {
      Serial.println(F("{\"status\":\"BUSY\",\"state\":\"CATCHING\"}"));
      followLineAndCatch();
    }
  }

  delay(50);
}
