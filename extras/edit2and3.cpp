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
    - Button 6: Spin RIGHT ~180 degrees in place (timed).
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
#define CMD_STAR        0x42 // Emergency Stop

// ── Speeds & Timing ───────────────────────────────────────────
#define SPEED_START_FAST  60   // Normal forward start speed (Button 1)
#define SPEED_START_SLOW  40   // Forward speed used after rotating (Button 2 & 3)
#define SPEED_ROTATE      50   // Fast speed for the blind swing-off at the start of a turn
#define SPEED_MIN         35   // Safety floor so speed never hits 0
#define TURN_CREEP_SPEED  35   // Slow creep speed while scent-seeking the line during a turn
#define CENTER_OFFSET_MS  110    // ms to drive forward after final junction (timed nudge)
#define CENTER_MAX_MS     800  // Safety cap for the sensor-based centering creep
#define STRAFE_MAX_MS     2000 // Safety cap for strafing sideways to the next line
#define TURN_BLIND_MS     250  // 90-degree blind "sensor break" to clear the current line
                               // 180-degree spin uses double this (TURN_BLIND_MS * 2) as its blind break

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
//  TURN 90 DEGREES RIGHT (Maintains Speed 60)
// ================================================================
bool rotateRight90() {
  Serial.println(F("\n--- Rotating 90 Degrees Right (Blind swing + slow creep) ---"));

  // --- STAGE 1: Blind Turn (Fast) ---
  // Aggressively swing off the current line so the mid sensor clears it.
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }
    setMotorSpeed(SPEED_ROTATE); // Fast blind swing-off
    car.Turn_Right();
  }

  // --- STAGE 2 & 3: Scent-Seeking Creep (Slow) + Mid-Sensor Lock ---
  // Creep slowly until the CENTER sensor locks onto the next black line.
  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }

    setMotorSpeed(TURN_CREEP_SPEED); // Slow creep to avoid momentum overshoot
    car.Turn_Right();

    if (digitalRead(SENSOR_RIGHT) == HIGH) {
      Serial.println(F("Right sensor clipped line! Stopping turn."));
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
  Serial.println(F("\n--- Rotating 90 Degrees Left (Blind swing + slow creep) ---"));

  // --- STAGE 1: Blind Turn (Fast) ---
  // Aggressively swing off the current line so the mid sensor clears it.
  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }
    setMotorSpeed(SPEED_ROTATE); // Fast blind swing-off
    car.Turn_Left();
  }

  // --- STAGE 2 & 3: Scent-Seeking Creep (Slow) + Mid-Sensor Lock ---
  // Creep slowly until the CENTER sensor locks onto the next black line.
  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
       Serial.println(F("E-STOP!")); restoreIR(); return false;
    }

    setMotorSpeed(TURN_CREEP_SPEED); // Slow creep to avoid momentum overshoot
    car.Turn_Left();

    if (digitalRead(SENSOR_LEFT) == HIGH) {
      Serial.println(F("Left sensor clipped line! Stopping turn."));
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
  unsigned long lastJunctionTime = 0; // Debounce: ignore re-counts of the same thick line

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
      if (!onJunction && (millis() - lastJunctionTime > 200)) {
        junctionCount++;
        lastJunctionTime = millis(); // Debounce timestamp for this junction
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
//  SPIN RIGHT ~180 DEGREES IN PLACE (timed, no sensors)
// ================================================================
void spinRight180() {
  Serial.println(F("\n--- Spinning Right ~180 degrees (double blind break + mid-sensor lock) ---"));

  // --- STAGE 1: Blind Turn (Fast) ---
  // Blind "sensor break" of DOUBLE the 90-degree duration so we swing well past
  // the half-turn point and clear the current line completely.
  unsigned long t_start = millis();
  while (millis() - t_start < (TURN_BLIND_MS * 2)) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
      Serial.println(F("E-STOP!")); restoreIR(); return;
    }
    setMotorSpeed(SPEED_ROTATE); // Fast blind swing
    car.Turn_Right();
  }

  // --- STAGE 2 & 3: Scent-Seeking Creep (Slow) + Mid-Sensor Lock ---
  // Keep spinning slowly until the CENTER sensor locks onto the line.
  while (true) {
    if (IrReceiver.decode() && IrReceiver.decodedIRData.command == CMD_STAR) {
      Serial.println(F("E-STOP!")); restoreIR(); return;
    }

    setMotorSpeed(TURN_CREEP_SPEED); // Slow creep to avoid momentum overshoot
    car.Turn_Right();

    if (digitalRead(SENSOR_MID) == HIGH) {
      Serial.println(F("Mid sensor locked on line! Stopping spin."));
      break;
    }
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- 180 Spin Completed. ---"));
  restoreIR();
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
  Serial.println(F("Press '6' to spin right ~180 degrees in place."));
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
    // Spin clockwise ~180 degrees in place (timed). Tune TURN_180_MS for an exact half-turn.
    spinRight180();
  }

  IrReceiver.resume();
}