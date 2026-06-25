/*
  ================================================================
  KS0560 Mecanum Car — FAST ROTATION + SLOW POST-TURN MOVEMENT
  ================================================================
  LOGIC: 
    - Button 1: Move 5 blocks forward starting at 60.
    - Button 2: Rotate Right at 60, then move 2 blocks starting at 40.
    - Button 3: Rotate Left at 60, then move 3 right-markers starting at 40.
    - Button 4: Advance one black line, center on it, then stop (repeat per press).
    - Button 5: Strafe LEFT to the next line, re-center, then stop (only when centered).
    - Button 6: Spin RIGHT ~180 degrees (sensor-based: skip 90-deg line, stop on 2nd
                line via MID/centerline sensor), then line-follow one block and center.
    - Button 7: Out 5 blocks (like button 1), spin RIGHT ~180, then 5 blocks back to
                the start (ends facing the reversed direction).
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
#define CMD_4           0x0C // TODO: confirm the IR code your remote sends for button 4
#define CMD_5           0x18 // TODO: confirm the IR code your remote sends for button 5
#define CMD_6           0x5E // TODO: confirm the IR code your remote sends for button 6
#define CMD_7           0x08 // TODO: confirm the IR code your remote sends for button 7
#define CMD_STAR        0x42 // Emergency Stop

// ── Speeds & Timing ───────────────────────────────────────────
#define SPEED_START_FAST  60   // Normal forward start speed (Button 1)
#define SPEED_START_SLOW  40   // Forward speed used after rotating (Button 2 & 3)
#define SPEED_ROTATE      60   // Maintained strong speed for 90-degree rotations
#define SPEED_MIN         35   // Safety floor so speed never hits 0
#define CENTER_OFFSET_MS  110    // ms to drive forward after final junction (timed nudge)
#define CENTER_MAX_MS     800  // Safety cap for the sensor-based centering creep
#define STRAFE_MAX_MS     2000 // Safety cap for strafing sideways to the next line
#define TURN_BLIND_MS     150  // Blind turn duration to clear the starting line
#define SPIN_180_MAX_MS   2000 // Safety cap for the sensor-based 180 spin (never spin forever)

// ── Turn Configuration ────────────────────────────────────────
#define STOP_SENSOR_RIGHT_TURN  SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN   SENSOR_LEFT
#define STOP_SENSOR_SPIN180     SENSOR_MID   // 180 spin counts on the centerline sensor →
                                             // its 2nd crossing is a true 180 (no offset/nudge).

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
//  the right sensor. Returns false if E-STOP was pressed.
// ================================================================
bool spinRight180() {
  Serial.println(F("\n--- Spinning Right ~180 degrees (MID sensor, Speed 60) ---"));

  // Phase 1: spin right until the MID sensor CLEARS the starting junction line.
  // This is sensor-based (not a fixed time) so the spin can never over-rotate
  // past the first arm — guaranteeing the next two crossings counted below are
  // the ~90 and ~180 degree arms, not the ~180 and ~270 ones.
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

  // Phase 2: keep spinning; count line crossings on the MID sensor. MID sits on
  // the centerline (no sideways offset), so its 2nd crossing lands at a true
  // ~180 degrees with no timing nudge. Skip the first (~90), stop on the second.
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
//  Drives out 5 blocks (exactly like button 1), spins right ~180 in
//  place (the button-6 spin, without its trailing 1-block move), then
//  drives 5 blocks back to the original position. Ends facing the
//  reversed direction. Aborts before the return leg if E-STOP is
//  pressed during the spin.
// ================================================================
void roundTrip180() {
  Serial.println(F("\n=== Round Trip: out 5, spin 180, back 5 ==="));

  moveForwardBlocks(5, SPEED_START_FAST);   // out leg (button 1)

  if (!spinRight180()) return;              // E-STOP during spin: stop here

  moveForwardBlocks(5, SPEED_START_FAST);   // return leg (button 1)

  Serial.println(F("=== Round Trip complete — back at start, facing reversed. ==="));
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);

  pinMode(SENSOR_LEFT,  INPUT);
  pinMode(SENSOR_MID,   INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  Serial.println(F("Ready."));
  Serial.println(F("Press '1' to go 5 blocks forward."));
  Serial.println(F("Press '2' to turn right and go 2 blocks."));
  Serial.println(F("Press '3' to turn left and go 3 right-markers."));
  Serial.println(F("Press '4' to advance to the next black line, then stop."));
  Serial.println(F("Press '5' to strafe left to the next line (only when centered)."));
  Serial.println(F("Press '6' to spin right ~180 (2nd line), then advance one block."));
  Serial.println(F("Press '7' to go out 5, spin 180, and return to the start."));
}

void loop() {
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
      moveRightSideMarkers(3, SPEED_START_SLOW); // Rotates at 60, but drives forward at 40!
    }
  } else if (cmd == CMD_4) {
    // Advance one black line: line-follow to the next full junction at speed 40,
    // then creep forward until only the MIDDLE sensor is on the line, then stop.
    // Press '4' again for the next line.
    moveForwardBlocks(1, SPEED_START_SLOW, true);
  } else if (cmd == CMD_5) {
    // Strafe LEFT (no rotation) to the next vertical line, then re-center so only
    // the middle sensor is on it. Runs only when currently centered. Press '5' to repeat.
    strafeLeftToNextLine(SPEED_START_SLOW);
  } else if (cmd == CMD_6) {
    // Sensor-based ~180: spin right past the 90-deg line, stop on the 2nd line
    // (right sensor), then line-follow forward one block and center on it.
    if (spinRight180()) {
      moveForwardBlocks(1, SPEED_START_SLOW, true);
    }
  } else if (cmd == CMD_7) {
    // Out 5 blocks, spin right ~180 in place, then 5 blocks back to the
    // original position (ends facing the opposite direction).
    roundTrip180();
  }

  IrReceiver.resume();
}