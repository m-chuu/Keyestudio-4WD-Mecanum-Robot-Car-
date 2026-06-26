/*
  ================================================================
  ButtonFunctions.cpp  (fileA)
  ----------------------------------------------------------------
  Extracted from Destination2.cpp.
  Keeps ONLY the action functions that each button triggers,
  plus the hardware setup (pins, defines, globals) and the small
  helpers those functions rely on. setup()/loop() were removed so
  this file can be reused as a function library by other sketches
  (see RobotSequences.cpp / fileB).

  Per-button functions kept:
    Button 1 -> moveForwardBlocks()
    Button 2 -> rotateRight90()
    Button 3 -> rotateLeft90() + moveRightSideMarkers()
    Button 4 -> moveForwardBlocks(..., sensorCenter = true)
    Button 5 -> strafeLeftToNextLine()
    Button 6 -> spinRight180()
    Button 7 -> roundTrip180()
  ================================================================
*/

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.hpp>

// ── Hardware pins ─────────────────────────────────────────────
#define RECV_PIN        A3
#define SENSOR_LEFT     A0
#define SENSOR_MID      A1
#define SENSOR_RIGHT    A2

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
#define CMD_STAR        0x42 // Emergency Stop

// ── Speeds & Timing ───────────────────────────────────────────
#define SPEED_START_FAST  60   // Normal forward start speed
#define SPEED_START_SLOW  40   // Forward speed used after rotating
#define SPEED_ROTATE      60   // Maintained strong speed for 90-degree rotations
#define SPEED_MIN         35   // Safety floor so speed never hits 0
#define CENTER_OFFSET_MS  110    // ms to drive forward after final junction (timed nudge)
#define CENTER_MAX_MS     800  // Safety cap for the sensor-based centering creep
#define STRAFE_MAX_MS     2000 // Safety cap for strafing sideways to the next line
#define TURN_BLIND_MS     150  // Blind turn duration to clear the starting line
#define SPIN_180_MAX_MS   2000 // Safety cap for the sensor-based 180 spin

// ── Turn Configuration ────────────────────────────────────────
#define STOP_SENSOR_RIGHT_TURN  SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN   SENSOR_LEFT
#define STOP_SENSOR_SPIN180     SENSOR_MID

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
//  UNIVERSAL FORWARD MOVEMENT (Accepts Customizable Start Speed)
// ================================================================
void moveForwardBlocks(int targetBlocks, uint8_t startSpeed, bool sensorCenter = false) {
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

  if (junctionCount == targetBlocks) {
    if (sensorCenter) {
      // Creep forward until ONLY the middle sensor is on the guide line
      // (i.e. past the perpendicular junction line), then stop.
      setMotorSpeed(SPEED_MIN);
      unsigned long t_creep = millis();
      while (millis() - t_creep < CENTER_MAX_MS) {
        uint8_t L = digitalRead(SENSOR_LEFT);
        uint8_t M = digitalRead(SENSOR_MID);
        uint8_t R = digitalRead(SENSOR_RIGHT);
        if (L == LOW && M == HIGH && R == LOW) break; // centered on guide line
        if (IrReceiver.decode()) {
          if (IrReceiver.decodedIRData.command == CMD_STAR) break;
          IrReceiver.resume();
        }
        car.Advance();
      }
    } else if (CENTER_OFFSET_MS > 0) {
      setMotorSpeed(SPEED_MIN);
      car.Advance();
      delay(CENTER_OFFSET_MS);
    }
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
//  STRAFE LEFT TO NEXT LINE (mecanum sideways, no rotation)
// ================================================================
void strafeLeftToNextLine(uint8_t strafeSpeed) {
  // Precondition: only run when centered on the guide line (only middle sensor).
  if (!(digitalRead(SENSOR_LEFT)  == LOW  &&
        digitalRead(SENSOR_MID)   == HIGH &&
        digitalRead(SENSOR_RIGHT) == LOW)) {
    Serial.println(F("\n--- Strafe Left ignored: not centered (need only middle sensor). ---"));
    restoreIR();
    return;
  }

  Serial.println(F("\n--- Strafing Left to next line ---"));

  // Phase 1: strafe left until the LEFT sensor reaches the next vertical line.
  unsigned long t0 = millis();
  while (digitalRead(SENSOR_LEFT) == LOW) {
    if (millis() - t0 > STRAFE_MAX_MS) {
      Serial.println(F("No line found (timeout). Stopping strafe."));
      restoreIR();
      return;
    }
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
      Serial.println(F("E-STOP!")); restoreIR(); return;
    }
    setMotorSpeed(strafeSpeed);
    car.L_Move();
  }
  Serial.println(F("Next line found on left. Re-centering."));

  // Phase 2: keep strafing left until re-centered (only the middle sensor on the line).
  t0 = millis();
  while (!(digitalRead(SENSOR_LEFT)  == LOW  &&
           digitalRead(SENSOR_MID)   == HIGH &&
           digitalRead(SENSOR_RIGHT) == LOW)) {
    if (millis() - t0 > CENTER_MAX_MS) break; // safety cap
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
      Serial.println(F("E-STOP!")); restoreIR(); return;
    }
    setMotorSpeed(strafeSpeed);
    car.L_Move();
  }

  Serial.println(F("--- Centered on next line. Strafe Left Completed. ---"));
  restoreIR();
}

// ================================================================
//  SPIN RIGHT ~180 DEGREES IN PLACE (sensor-based, Speed 60)
//  Starts centered on a junction. Spins right, skips the line found
//  near 90 degrees, and stops on the SECOND line (~180 deg) seen by
//  the MID sensor. Returns false if E-STOP was pressed.
// ================================================================
bool spinRight180() {
  Serial.println(F("\n--- Spinning Right ~180 degrees (MID sensor, Speed 60) ---"));

  // Phase 1: spin right until the MID sensor CLEARS the starting junction line.
  unsigned long t_guard = millis();
  while (digitalRead(STOP_SENSOR_SPIN180) == HIGH) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
      Serial.println(F("E-STOP!")); restoreIR(); return false;
    }
    if (millis() - t_guard > SPIN_180_MAX_MS) {   // safety: line never cleared
      Serial.println(F("180 spin: start line never cleared — stopping."));
      break;
    }
    setMotorSpeed(SPEED_ROTATE); // Spin clockwise in place at 60
    car.Turn_Right();
  }

  // Phase 2: keep spinning; count line crossings on the MID sensor.
  // Skip the first (~90), stop on the second (~180).
  int  lineCount = 0;
  bool onLine = false;          // Phase 1 already cleared the start line.
  t_guard = millis();

  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
      Serial.println(F("E-STOP!")); restoreIR(); return false;
    }

    // Safety cap: never spin forever if a line is missed.
    if (millis() - t_guard > SPIN_180_MAX_MS) {
      Serial.println(F("180 spin timeout — stopping."));
      break;
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();

    if (digitalRead(STOP_SENSOR_SPIN180) == HIGH) {
      if (!onLine) {            // rising edge: just reached a new line
        onLine = true;
        lineCount++;
        Serial.print(F("Line crossing: "));
        Serial.println(lineCount);
        if (lineCount >= 2) {   // second line ≈ 180 degrees
          Serial.println(F("Second line found — 180 reached."));
          break;
        }
      }
    } else {
      onLine = false;           // sensor cleared the line; ready for next crossing
    }
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- 180 Spin Completed. ---"));
  return true;
}

// ================================================================
//  ROUND TRIP: FORWARD 5, SPIN 180, FORWARD 5 (back to start)
// ================================================================
void roundTrip180() {
  Serial.println(F("\n=== Round Trip: out 5, spin 180, back 5 ==="));

  moveForwardBlocks(5, SPEED_START_FAST);   // out leg (button 1)

  if (!spinRight180()) return;              // E-STOP during spin: stop here

  moveForwardBlocks(5, SPEED_START_FAST);   // return leg (button 1)

  Serial.println(F("=== Round Trip complete — back at start, facing reversed. ==="));
}
