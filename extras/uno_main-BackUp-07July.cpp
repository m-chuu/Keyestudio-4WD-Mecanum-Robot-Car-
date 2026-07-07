/*
  ================================================================
  KS0560 Mecanum Car — FULL RUN + FAVORIOT STATUS REPORTING
  ================================================================
  LOGIC:
    - Button 1: Move 5 blocks forward starting at 60.
    - Button 2: Rotate Right at 60, then move 2 blocks starting at 40.
    - Button 3: Rotate Left at 60, then move 3 right-markers starting at 40.
    - Button 4: Move 6 blocks forward, Creep and Catch, turn 180°,
                go 7 blocks, backup, then SEND STATUS TO FAVORIOT.
    - Button 5: Move 4 blocks forward, Rotate Left, Rotate Right, Move 2 blocks forward, Creep and Catch.
    - Button 6: Move 4 blocks forward, Rotate Right, Rotate Left, Move 2 blocks forward, Creep and Catch.
    - Button 7: Rotate right, go 2 blocks, rotate right, go 8 blocks, then backup.
    - Button 8: Rotate left, follow 2 right-markers, rotate left, follow 8 right-markers, then backup.

  FAVORIOT LINK:
    Uno pin 10 (TX) -> ESP32 RX2 (16), Uno pin 11 (RX) <- ESP32 TX2 (17).
    The ESP32 wraps whatever it receives into
    {"data":{"distance": <payload>}} and POSTs it to Favoriot,
    so the status is sent as a quoted string to keep the JSON valid.
  ================================================================
*/

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.hpp>
#include <Servo.h>
#include <SoftwareSerial.h>

// ── Hardware pins ─────────────────────────────────────────────
#define RECV_PIN        A3
#define SENSOR_LEFT     A0
#define SENSOR_MID      A1
#define SENSOR_RIGHT    A2

// ── SERVO & ULTRASONIC PINS ───────────────────────────────────
const uint8_t SERVO_PIN           = 9;
const uint8_t ULTRASONIC_TRIG_PIN = 12;
const uint8_t ULTRASONIC_ECHO_PIN = 13;

// ── ESP32 LINK (Favoriot bridge) ──────────────────────────────
SoftwareSerial espSerial(11, 10);  // RX, TX

// ── EXTERN LIBRARY VARIABLES ──────────────────────────────────
extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

// ── IR Codes ──────────────────────────────────────────────────
#define CMD_1           0x16
#define CMD_2           0x19
#define CMD_3           0x0D
#define CMD_4           0x0C
#define CMD_5           0x18
#define CMD_6           0x5E
#define CMD_7           0x08
#define CMD_8           0x1C
#define CMD_STAR        0x42 // Emergency Stop

// ── Speeds & Timing ───────────────────────────────────────────
#define SPEED_START_FAST  60   // Normal forward start speed (Button 1)
#define SPEED_START_SLOW  40   // Forward speed used after rotating (Button 2 & 3)
#define SPEED_ROTATE      60   // Maintained strong speed for 90-degree rotations
#define SPEED_MIN         35   // Safety floor so speed never hits 0
#define CENTER_OFFSET_MS  110    // ms to drive forward after final junction
#define TURN_BLIND_MS     150  // Blind turn duration to clear the starting line
#define BACKWARD_OFFSET_MS 3000 // ms to move backward after catching object (TESTING: increased from 220)

// ── SERVO ANGLES ──────────────────────────────────────────────
const int SERVO_OPEN_ANGLE   = 15;
const int SERVO_CLOSED_ANGLE = 85;

// ── OBJECT DETECTION ──────────────────────────────────────────
const uint16_t OBJECT_DETECT_DISTANCE_CM = 4;
const unsigned long ULTRASONIC_CHECK_INTERVAL_MS = 50;

// ── SERVO & ULTRASONIC GLOBALS ────────────────────────────────
Servo servoClamp;
uint16_t currentDistanceCm = 0;
unsigned long lastUltrasonicCheckTime = 0;
unsigned long lastPrintTime = 0;

// ── Turn Configuration ────────────────────────────────────────
#define STOP_SENSOR_RIGHT_TURN  SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN   SENSOR_LEFT

mecanumCar car(3, 2);

// ================================================================
//  HELPERS
// ================================================================
void setMotorSpeed(uint8_t spd) {
  speed_Upper_L = spd;
  speed_Lower_L = spd;
  speed_Upper_R = spd;
  speed_Lower_R = spd;
}

void restoreIR() {
  car.Stop();
  IrReceiver.begin(RECV_PIN, false);
}

// ================================================================
//  FAVORIOT STATUS (via ESP32 bridge)
// ================================================================
// Sends a quoted string so the ESP32's payload
// {"data":{"distance": "<status>"}} stays valid JSON.
void sendStatusToFavoriot(const char* status) {
  espSerial.print('"');
  espSerial.print(status);
  espSerial.println('"');

  Serial.print(F("[FAVORIOT] Status sent to ESP32: "));
  Serial.println(status);
}

// ================================================================
//  SERVO & ULTRASONIC FUNCTIONS
// ================================================================
uint16_t readDistanceCmFast() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  unsigned long echoUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000UL);
  if (echoUs == 0) return 0;
  return echoUs / 58;
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

bool checkUltrasonicNonBlocking() {
  unsigned long now = millis();
  if (now - lastUltrasonicCheckTime < ULTRASONIC_CHECK_INTERVAL_MS) return false;
  lastUltrasonicCheckTime = now;
  currentDistanceCm = readDistanceCmFast();
  if (currentDistanceCm > 0 && currentDistanceCm <= OBJECT_DETECT_DISTANCE_CM) return true;
  return false;
}

// ================================================================
//  CREEP & CATCH (Continuous ultrasonic monitoring with real-time output)
// ================================================================
bool creepAndCatch() {
  Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║     CREEPING FORWARD TO CATCH OBJECT (4cm)        ║"));
  Serial.println(F("╚════════════════════════════════════════════════════╝"));

  setMotorSpeed(SPEED_MIN);
  unsigned long lastPrintTime = 0;

  while (true) {
    // Check for E-STOP
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
      Serial.println(F("E-STOP during creep!"));
      car.Stop();
      restoreIR();
      return false;
    }

    // Move forward continuously
    car.Advance();

    // Read ultrasonic EVERY LOOP (no blocking checks)
    currentDistanceCm = readDistanceCmFast();

    // Print distance every 100ms for debugging
    unsigned long now = millis();
    if (now - lastPrintTime >= 100) {
      lastPrintTime = now;
      Serial.print(F("[DISTANCE] "));
      Serial.print(currentDistanceCm);
      Serial.println(F(" cm"));
    }

    // Detect object at 4cm or closer
    if (currentDistanceCm > 0 && currentDistanceCm <= OBJECT_DETECT_DISTANCE_CM) {
      Serial.println(F(""));
      Serial.println(F("╔════════════════════════════════════════════════════╗"));
      Serial.println(F("║       ⚠ OBJECT DETECTED AT 4 CM OR CLOSER ⚠     ║"));
      Serial.println(F("║         Stopping robot and closing servo...       ║"));
      Serial.println(F("╚════════════════════════════════════════════════════╝"));
      car.Stop();
      delay(100);
      servoClosed();
      delay(200);
      return true;
    }
  }
}

// ================================================================
//  BACKWARD MOVEMENT (Clears object after servo opens)
// ================================================================
void moveBackwardTimed(uint8_t speed) {
  servoOpen();

  Serial.print(F("\n--- Moving Backward (Speed "));
  Serial.print(speed);
  Serial.println(F(") ---"));

  setMotorSpeed(speed);
  unsigned long t_start = millis();
  while (millis() - t_start < BACKWARD_OFFSET_MS) {
    car.Back();

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) {
        Serial.println(F("E-STOP during backward!"));
        break;
      }
      IrReceiver.resume();
    }
  }

  car.Stop();
  Serial.println(F("--- Movement Completed. ---"));

  restoreIR();
}

// ================================================================
//  TURN 90 DEGREES RIGHT (Maintains Speed 60)
// ================================================================
bool rotateRight90() {
  Serial.println(F("\n--- Rotating 90 Degrees Right (Speed 60) ---"));

  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }
    setMotorSpeed(SPEED_ROTATE); // Maintained at 60
    car.Turn_Right();
  }

  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();

    if (digitalRead(STOP_SENSOR_RIGHT_TURN) == HIGH) {
      Serial.println(F("Line found! Stopping turn."));
      break;
    }
  }

  car.Stop();
  delay(200);
  return true;
}

// ================================================================
//  TURN 90 DEGREES LEFT (Maintains Speed 60)
// ================================================================
bool rotateLeft90() {
  Serial.println(F("\n--- Rotating 90 Degrees Left (Speed 60) ---"));

  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }
    setMotorSpeed(SPEED_ROTATE); // Maintained at 60
    car.Turn_Left();
  }

  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();

    if (digitalRead(STOP_SENSOR_LEFT_TURN) == HIGH) {
      Serial.println(F("Line found! Stopping turn."));
      break;
    }
  }

  car.Stop();
  delay(200);
  return true;
}

// ================================================================
//  TURN 180 DEGREES RIGHT (Two 90-Degree Turns)
// ================================================================
bool rotateRight180() {
  Serial.println(F("\n--- Rotating 180 Degrees Right ---"));

  if (!rotateRight90()) {
    Serial.println(F("180-degree turn aborted: first 90-degree turn failed."));
    return false;
  }

  if (!rotateRight90()) {
    Serial.println(F("180-degree turn aborted: second 90-degree turn failed."));
    return false;
  }

  Serial.println(F("180-degree turn completed successfully."));
  return true;
}

// ================================================================
//  UNIVERSAL FORWARD MOVEMENT (Accepts Customizable Start Speed)
// ================================================================
void moveForwardBlocks(int targetBlocks, uint8_t startSpeed) {
  int junctionCount = 0;
  bool onJunction = true;

  Serial.print(F("\n--- Moving Forward "));
  Serial.print(targetBlocks);
  Serial.print(F(" Blocks (Starting at Speed "));
  Serial.print(startSpeed);
  Serial.println(F(") ---"));

  while (junctionCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    // --- DYNAMIC DECELERATION FORMULA ---
    int calculatedSpeed = startSpeed - (junctionCount * 5);
    if (calculatedSpeed < SPEED_MIN) {
      calculatedSpeed = SPEED_MIN;
    }
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (L == HIGH && M == HIGH && R == HIGH) {
      if (!onJunction) {
        junctionCount++;
        Serial.print(F("Junction Crossed: "));
        Serial.print(junctionCount);
        Serial.print(F(" | Speed: "));
        Serial.println(calculatedSpeed);
        onJunction = true;
        if (junctionCount == targetBlocks) break;
      }
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH) {
      onJunction = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Right();
    }

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) {
        Serial.println(F("E-STOP PRESSED!"));
        break;
      }
      IrReceiver.resume();
    }
  }

  if (junctionCount == targetBlocks && CENTER_OFFSET_MS > 0) {
    setMotorSpeed(SPEED_MIN);
    car.Advance();
    delay(CENTER_OFFSET_MS);
  }

  Serial.println(F("--- Movement Completed. ---"));
  restoreIR();
}

// ================================================================
//  RIGHT-SENSOR BIAS MOVEMENT (Accepts Customizable Start Speed)
// ================================================================
void moveRightSideMarkers(int targetBlocks, uint8_t startSpeed) {
  int markerCount = 0;
  bool onMarker = true;

  Serial.print(F("\n--- Watching Right Sensor for "));
  Serial.print(targetBlocks);
  Serial.print(F(" Markers (Starting at Speed "));
  Serial.print(startSpeed);
  Serial.println(F(") ---"));

  while (markerCount < targetBlocks) {
    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    // --- DYNAMIC DECELERATION FORMULA ---
    int calculatedSpeed = startSpeed - (markerCount * 5);
    if (calculatedSpeed < SPEED_MIN) {
      calculatedSpeed = SPEED_MIN;
    }
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (M == HIGH && R == HIGH) {
      if (!onMarker) {
        markerCount++;
        Serial.print(F("Right Marker Crossed: "));
        Serial.print(markerCount);
        Serial.print(F(" | Speed: "));
        Serial.println(calculatedSpeed);
        onMarker = true;
        if (markerCount == targetBlocks) break;
      }
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW) {
      onMarker = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH) {
      onMarker = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW) {
      onMarker = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW) {
      onMarker = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW) {
      onMarker = false;
      setMotorSpeed(currentSpeed + 10);
      car.Turn_Left();
    }

    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == CMD_STAR) {
        Serial.println(F("E-STOP PRESSED!"));
        break;
      }
      IrReceiver.resume();
    }
  }

  if (markerCount == targetBlocks && CENTER_OFFSET_MS > 0) {
    setMotorSpeed(SPEED_MIN);
    car.Advance();
    delay(CENTER_OFFSET_MS);
  }

  Serial.println(F("--- Right Marker Movement Completed. ---"));
  restoreIR();
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);   // link to ESP32 (Favoriot bridge)

  pinMode(SENSOR_LEFT,  INPUT);
  pinMode(SENSOR_MID,   INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  servoClamp.attach(SERVO_PIN);
  servoOpen();

  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  Serial.println(F("Ready."));
  Serial.println(F("Press '1' to go 5 blocks forward."));
  Serial.println(F("Press '2' to turn right and go 2 blocks."));
  Serial.println(F("Press '3' to turn left and go 3 right-markers."));
  Serial.println(F("Press '4' to go 6 blocks forward, turn 180°, go 7 blocks, backup, then report to Favoriot."));
  Serial.println(F("Press '5' to move 4 blocks forward, Rotate Right, Rotate Left, Move 2 blocks forward, Creep and Catch."));
  Serial.println(F("Press '6' to move 4 blocks forward, Rotate Left, Rotate Right, Move 2 blocks forward, Creep and Catch."));
  Serial.println(F("Press '7' to rotate right, go 2 blocks, rotate right, go 8 blocks, then backup."));
  Serial.println(F("Press '8' to rotate left, follow 2 right-markers, rotate left, follow 8 right-markers, then backup."));
}

void loop() {
  // Relay any ESP32 replies (e.g. "Favoriot OK: 201") to the USB monitor
  while (espSerial.available()) {
    Serial.write(espSerial.read());
  }

  if (!IrReceiver.decode()) return;

  uint8_t cmd = IrReceiver.decodedIRData.command;

  if (cmd == 0x00 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    IrReceiver.resume();
    return;
  }

  if (cmd == CMD_1) {
    moveForwardBlocks(5, SPEED_START_FAST); // Starts straight out at 60
  } else if (cmd == CMD_2) {
    if (rotateRight90()) {
      moveForwardBlocks(2, SPEED_START_SLOW); // Rotates at 60, but drives forward at 40!
    }
  } else if (cmd == CMD_3) {
    if (rotateLeft90()) {
      moveRightSideMarkers(2, SPEED_START_SLOW); // Rotates at 60, but drives forward at 40!
    }
  } else if (cmd == CMD_4) {
    // 1) Ensure servo is open
    servoOpen();
    // 2) Move forward 6 blocks
    moveForwardBlocks(6, SPEED_START_FAST);
    // 3) Creep forward and catch object (with ultrasonic detection)
    if (creepAndCatch()) {
      // 4) Turn 180 degrees right
      if (rotateRight90() && rotateRight90()) {
        // 5) Move forward 7 blocks
        moveForwardBlocks(7, SPEED_START_FAST);
        // 6) Move backward to clear object
        moveBackwardTimed(SPEED_MIN);
        // 7) Task complete — report status to Favoriot via ESP32
        sendStatusToFavoriot("task4_complete");
      }
    }
  } else if (cmd == CMD_5) {
    // 1) Ensure servo is open before starting the run
    servoOpen();
    moveForwardBlocks(4, SPEED_START_FAST);
    if (rotateLeft90()) {
      moveRightSideMarkers(2, SPEED_START_SLOW);
      if (rotateRight90()) {
        moveForwardBlocks(2, SPEED_START_SLOW);
        delay(1000); // 1 second pause

        // 2) Creep forward and catch object
        creepAndCatch();
      }
    }
  } else if (cmd == CMD_6) {
    moveForwardBlocks(4, SPEED_START_FAST);
    if (rotateRight90()) {
      moveForwardBlocks(2, SPEED_START_SLOW);
      if (rotateLeft90()) {
        moveRightSideMarkers(2, SPEED_START_SLOW);
        delay(1000); // 1 second pause

        // 2) Creep forward and catch object
        creepAndCatch();
      }
    }
  } else if (cmd == CMD_7) {
    if (rotateRight90()) {
      moveForwardBlocks(2, SPEED_START_SLOW);
      if (rotateRight90()) {
        moveForwardBlocks(7, SPEED_START_SLOW);
        moveBackwardTimed(SPEED_MIN);
      }
    }
  } else if (cmd == CMD_8) {
    if (rotateLeft90()) {
      moveRightSideMarkers(2, SPEED_START_SLOW);
      if (rotateLeft90()) {
        moveRightSideMarkers(7, SPEED_START_SLOW);
        moveBackwardTimed(SPEED_MIN);
      }
    }
  }
  IrReceiver.resume();
}
