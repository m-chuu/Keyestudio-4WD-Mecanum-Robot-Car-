/*
  ================================================================
  IR + SERVO CLAMP + ULTRASONIC SENSOR CONTROL
  ================================================================
  Flow:
    1. Servo OPEN at startup
    2. Press IR Button to start object detection
    3. Ultrasonic monitors for object at 4cm
    4. When object detected → Wait for Serial command
    5. Send 'c' or '1' via Serial Monitor to CLOSE clamp
    6. Send 'o' or '2' via Serial Monitor to OPEN clamp

  IR Remote Buttons (NEC protocol):
    - Button 5 (CMD_5 = 0x18): Start object detection
    - Button * (CMD_STAR = 0x42): Emergency stop / Reset
  ================================================================
*/

#include <Arduino.h>
#include <Servo.h>
#define DECODE_NEC
#include <IRremote.hpp>

// ── PIN DEFINITIONS ───────────────────────────────────────────
const uint8_t SERVO_PIN           = 9;    // Clamp servo signal
const uint8_t ULTRASONIC_TRIG_PIN = 12;   // Ultrasonic trigger
const uint8_t ULTRASONIC_ECHO_PIN = 13;   // Ultrasonic echo
const uint8_t IR_RECV_PIN         = A3;   // IR remote receiver

// ── SERVO ANGLES ──────────────────────────────────────────────
const int SERVO_OPEN_ANGLE   = 15;   // Fully open position
const int SERVO_CLOSED_ANGLE = 85;   // Fully closed (grip)

// ── IR REMOTE CODES ───────────────────────────────────────────
const uint8_t CMD_5           = 0x18;   // Start object detection
const uint8_t CMD_STAR        = 0x42;   // Emergency stop / Reset

// ── DETECTION THRESHOLD ───────────────────────────────────────
const uint16_t OBJECT_DETECT_DISTANCE_CM = 4;  // Trigger distance

// ── TIMING CONSTANTS ──────────────────────────────────────────
const unsigned long ULTRASONIC_READ_INTERVAL_MS = 200;  // Read every 200ms
const unsigned long DISPLAY_INTERVAL_MS         = 500;  // Update display every 500ms

// ── SYSTEM STATES ──────────────────────────────────────────────
enum SystemState {
  STATE_IDLE,                  // Waiting for IR command
  STATE_DETECTING,             // Monitoring for object
  STATE_OBJECT_DETECTED,       // Object found, waiting for catch command
  STATE_CATCHING,              // Closing servo
  STATE_CAUGHT,                // Servo closed, object caught
  STATE_RELEASING              // Opening servo
};

// ── GLOBALS ───────────────────────────────────────────────────
Servo servoClamp;
SystemState systemState = STATE_IDLE;

uint16_t currentDistanceCm = 0;
uint16_t closestDistanceCm = 999;

unsigned long lastUltrasonicReadTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long objectDetectedTime = 0;

bool servoResponding = false;
bool ultrasonicResponding = false;

// ================================================================
//  ULTRASONIC DISTANCE CALCULATION
// ================================================================
uint16_t readDistanceCm() {
  // Send 10µs pulse to trigger pin
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // Measure echo pulse duration (max 25ms timeout)
  unsigned long echoUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000UL);

  // Convert to distance: echoUs / 58 ≈ distance in cm
  if (echoUs == 0) {
    return 0;  // Timeout
  }

  uint16_t distanceCm = echoUs / 58;
  return distanceCm;
}

// ================================================================
//  SERVO CONTROL
// ================================================================
void servoOpen() {
  servoClamp.write(SERVO_OPEN_ANGLE);
  servoResponding = true;
  delay(500);  // Wait for servo movement
  Serial.println(F("\n[SERVO] → OPEN (15°)"));
}

void servoClosed() {
  servoClamp.write(SERVO_CLOSED_ANGLE);
  servoResponding = true;
  delay(500);  // Wait for servo movement
  Serial.println(F("\n[SERVO] → CLOSED (85°) - Object Caught!"));
}

// ================================================================
//  UPDATE ULTRASONIC READING
// ================================================================
void updateUltrasonicReading() {
  unsigned long now = millis();

  if (now - lastUltrasonicReadTime < ULTRASONIC_READ_INTERVAL_MS) {
    return;
  }

  lastUltrasonicReadTime = now;

  currentDistanceCm = readDistanceCm();
  if (currentDistanceCm > 0) {
    ultrasonicResponding = true;

    // Track closest distance
    if (currentDistanceCm < closestDistanceCm) {
      closestDistanceCm = currentDistanceCm;
    }

    // Check for object at target distance
    if (systemState == STATE_DETECTING && currentDistanceCm <= OBJECT_DETECT_DISTANCE_CM) {
      systemState = STATE_OBJECT_DETECTED;
      objectDetectedTime = now;
      Serial.println(F(""));
      Serial.println(F("╔════════════════════════════════════════════════════╗"));
      Serial.println(F("║       OBJECT DETECTED AT 4 CM!                    ║"));
      Serial.println(F("║   Send 'c' or '1' via Serial to CLOSE clamp      ║"));
      Serial.println(F("╚════════════════════════════════════════════════════╝"));
    }
  }
}

// ================================================================
//  HANDLE IR REMOTE COMMANDS
// ================================================================
void handleIRCommand(uint8_t cmd) {
  if (cmd == CMD_5) {
    Serial.println(F("\n[IR] Button 5 pressed → Starting object detection..."));
    systemState = STATE_DETECTING;
    closestDistanceCm = 999;
    currentDistanceCm = 0;
  }
  else if (cmd == CMD_STAR) {
    Serial.println(F("\n[IR] Emergency STOP → Resetting to IDLE state"));
    systemState = STATE_IDLE;
    servoOpen();
  }
}

// ================================================================
//  HANDLE SERIAL MONITOR COMMANDS
// ================================================================
void handleSerialCommand() {
  if (!Serial.available()) {
    return;
  }

  char cmd = Serial.read();
  Serial.println(cmd);  // Echo the command

  if (cmd == 'c' || cmd == '1') {
    if (systemState == STATE_OBJECT_DETECTED) {
      Serial.println(F("[SERIAL] Close command received → CATCHING object!"));
      systemState = STATE_CATCHING;
      servoClosed();
      systemState = STATE_CAUGHT;
    } else {
      Serial.println(F("[SERIAL] Not in detection mode. Press IR Button 5 first."));
    }
  }
  else if (cmd == 'o' || cmd == '2') {
    if (systemState == STATE_CAUGHT) {
      Serial.println(F("[SERIAL] Open command received → RELEASING object!"));
      systemState = STATE_RELEASING;
      servoOpen();
      systemState = STATE_IDLE;
    } else {
      Serial.println(F("[SERIAL] No object caught. Start detection first."));
    }
  }
  else if (cmd == 'r' || cmd == '3') {
    Serial.println(F("[SERIAL] Reset command → Back to IDLE"));
    systemState = STATE_IDLE;
    servoOpen();
  }
  else if (cmd == 's' || cmd == '?') {
    Serial.println(F("\n[COMMANDS]"));
    Serial.println(F("  IR Button 5: Start object detection"));
    Serial.println(F("  IR Button *: Emergency stop"));
    Serial.println(F("  Serial 'c' or '1': Close clamp"));
    Serial.println(F("  Serial 'o' or '2': Open clamp"));
    Serial.println(F("  Serial 'r' or '3': Reset system"));
    Serial.println(F("  Serial 's' or '?': Show this help"));
  }
}

// ================================================================
//  FORMATTED DISPLAY OUTPUT
// ================================================================
void displayStatus() {
  unsigned long now = millis();

  if (now - lastDisplayTime < DISPLAY_INTERVAL_MS) {
    return;
  }

  lastDisplayTime = now;

  // System state name
  const char* stateName;
  switch (systemState) {
    case STATE_IDLE:              stateName = "IDLE"; break;
    case STATE_DETECTING:         stateName = "DETECTING"; break;
    case STATE_OBJECT_DETECTED:   stateName = "OBJECT DETECTED [!]"; break;
    case STATE_CATCHING:          stateName = "CATCHING..."; break;
    case STATE_CAUGHT:            stateName = "CAUGHT [OK]"; break;
    case STATE_RELEASING:         stateName = "RELEASING..."; break;
    default:                      stateName = "UNKNOWN"; break;
  }

  // ── SYSTEM STATE ──
  Serial.print(F("\r[STATE] "));
  Serial.print(stateName);
  Serial.print(F("  |  "));

  // ── SERVO STATUS ──
  Serial.print(F("[SERVO] "));
  if (servoResponding) {
    Serial.print(F("[OK] "));
  } else {
    Serial.print(F("[FAIL] "));
  }

  // ── ULTRASONIC STATUS ──
  Serial.print(F(" [ULTRASONIC] "));
  if (ultrasonicResponding) {
    Serial.print(currentDistanceCm);
    Serial.print(F(" cm"));
  } else {
    Serial.print(F("No signal"));
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }

  delay(500);

  // Initialize Servo
  servoClamp.attach(SERVO_PIN);
  servoClamp.write(SERVO_OPEN_ANGLE);
  servoResponding = true;

  // Initialize Ultrasonic Pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // Initialize IR Receiver
  IrReceiver.begin(IR_RECV_PIN, DISABLE_LED_FEEDBACK);

  Serial.println(F(""));
  Serial.println(F("╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║   SERVO CLAMP + ULTRASONIC OBJECT CATCHER v1.0    ║"));
  Serial.println(F("╚════════════════════════════════════════════════════╝"));
  Serial.println(F(""));
  Serial.println(F("STARTUP SEQUENCE:"));
  Serial.println(F("  [OK] Servo initialized (OPEN)"));
  Serial.println(F("  [OK] Ultrasonic ready"));
  Serial.println(F("  [OK] IR receiver ready"));
  Serial.println(F(""));
  Serial.println(F("NEXT STEPS:"));
  Serial.println(F("  1. Press IR Button 5 to start object detection"));
  Serial.println(F("  2. Bring object within 4 cm"));
  Serial.println(F("  3. Send 'c' or '1' via Serial Monitor to catch"));
  Serial.println(F(""));

  delay(1000);
}

// ================================================================
//  LOOP: Main control flow
// ================================================================
void loop() {
  // Handle IR remote commands
  if (IrReceiver.decode()) {
    uint8_t cmd = IrReceiver.decodedIRData.command;
    if (cmd != 0x00 && !(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      handleIRCommand(cmd);
    }
    IrReceiver.resume();
  }

  // Handle Serial Monitor commands
  handleSerialCommand();

  // Update ultrasonic reading (only if detecting)
  if (systemState == STATE_DETECTING || systemState == STATE_OBJECT_DETECTED) {
    updateUltrasonicReading();
  }

  // Display status
  displayStatus();

  delay(50);
}
