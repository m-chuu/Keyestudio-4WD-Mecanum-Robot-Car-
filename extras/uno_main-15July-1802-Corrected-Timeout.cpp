/*
  ================================================================
  KS0560 Mecanum Car — COMPRESSED SEQUENCE CONTROL WITH REVERSE
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

// ── Ultrasonic + Gripper pins ─────────────────────────────────
#define TRIG_PIN        12
#define ECHO_PIN        13
#define GRIPPER_PIN     9

#define GRIPPER_OPEN_ANGLE   20
#define GRIPPER_CLOSE_ANGLE  90
#define GRAB_DISTANCE_CM     3

// Time to let the servo physically reach the open angle before
// cutting power (limp mode). write() returns instantly; the servo
// needs real time to travel 100° -> 30° or it goes limp mid-move.
#define LIMP_SETTLE_MS       300

Servo gripperServo;
bool gripperIsOpen = false;

// Color pins
#define COLOR_OUT 6
#define COLOR_S2  7
#define COLOR_S3  8

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
#define CMD_STAR        0x42


// ── Speeds, Timing & Tuning ───────────────────────────────────
#define SPEED_START_FAST  60   
#define SPEED_START_SLOW  40   //38
#define SPEED_ROTATE      60   //52 //48
#define SPEED_MIN         35   
#define CENTER_OFFSET_MS  110  
#define TURN_BLIND_MS     150 //180 //200
#define QUICK_REVERSE_MS  250  // <-- TIME IN MILLISECONDS TO REVERSE BEFORE TURNING
#define MOVE_FWD_TIMEOUT_MS 7500  // Max time for MOVE_FORWARD_TIMED before giving up
#define DRIFT_LEFT_TIMEOUT_MS 1500 // Max time for DRIFT_LEFT_TIMED before giving up
#define ROTATE_TIMEOUT_MS 5000     // Max time for a 90-degree rotation before giving up
#define OBJECT_SLOW_DISTANCE_CM  10
#define OBJECT_STOP_DISTANCE_CM   4
#define OBJECT_APPROACH_SPEED    28

#define STOP_SENSOR_RIGHT_TURN  SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN   SENSOR_LEFT

mecanumCar car(3, 2);
SoftwareSerial espSerial(11, 10);

volatile bool cloudEmergencyStop = false;

// Set by moveLineTracking when a timed move gives up before reaching
// its marker count; read by REVERSE_IF_TIMEOUT steps.
bool moveTimedOut = false;

// ── SEQUENCE ENGINE DEFINITIONS ───────────────────────────────
enum ActionType {
  MOVE_FORWARD,
  MOVE_FORWARD_TIMED,
  MOVE_BACKWARD,
  MOVE_LEFT_MARKERS,
  MOVE_RIGHT_MARKERS,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  ROTATE_180,
  DRIFT_LEFT,
  DRIFT_LEFT_TIMED,
  DETECT_GRAB,
  DETECT_BLUE_GRAB,
  DETECT_RED_GRAB,
  DETECT_YELLOW_GRAB,
  OPEN_GRIPPER,
  DELAY_MS,
  QUICK_REVERSE,
  REVERSE_TIME,
  REVERSE_IF_TIMEOUT,
  STOP_WHEELS
};

struct Step {
  ActionType action;
  int param1;      // Distance / Blocks / Duration
  uint8_t param2;  // Speed
};

// ================================================================
//  HELPERS
// ================================================================
void setMotorSpeed(uint8_t spd) {
  speed_Upper_L = spd; speed_Lower_L = spd;
  speed_Upper_R = spd; speed_Lower_R = spd;
}

void setMotorSpeedLR(uint8_t leftSpeed, uint8_t rightSpeed) {
  speed_Upper_L = leftSpeed;
  speed_Lower_L = leftSpeed;

  speed_Upper_R = rightSpeed;
  speed_Lower_R = rightSpeed;
}

void restoreIR() {
  car.Stop();
  IrReceiver.begin(RECV_PIN, false);
}

void sendStatusToFavoriot(const char* status) {
  espSerial.print('"');
  espSerial.print(status);
  espSerial.println('"');

  Serial.print(F("[FAVORIOT] Status sent to ESP32: "));
  Serial.println(status);
}

long readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return (duration == 0) ? 999 : duration / 58;
}

void openGripper() {
  if (!gripperServo.attached()) gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  gripperIsOpen = true;
  delay(100);

  // Safe to go limp here — nothing is being gripped, no load to hold.
  // Both gripper functions re-attach on demand before the next move.
  delay(LIMP_SETTLE_MS);
  gripperServo.detach();
}

void closeGripper() {
  if (!gripperServo.attached()) gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(GRIPPER_CLOSE_ANGLE);
  gripperIsOpen = false;
  delay(100);
  // Intentionally NOT detaching here — the object is being carried,
  // the servo must stay powered to hold the grip under load.
}

bool checkEStop()
{
    // Check ESP32 cloud commands during every movement loop.
    // This makes Favoriot STOP work while CMD4/CMD5/CMD6 is running.
    if (espSerial.available())
    {
        String cloudCmd = espSerial.readStringUntil('\n');
        cloudCmd.replace("\r", "");
        cloudCmd.trim();
        cloudCmd.toUpperCase();

        if (cloudCmd == "STOP" ||
            cloudCmd == "STAR" ||
            cloudCmd == "CMDSTAR" ||
            cloudCmd == "EMERGENCY_STOP")
        {
            Serial.println(F("!! CLOUD EMERGENCY STOP !!"));
            cloudEmergencyStop = false;
            car.Stop();
            restoreIR();
            return true;
        }

        // A non-stop cloud command received during a running mission is ignored.
        Serial.print(F("Cloud command ignored while busy: "));
        Serial.println(cloudCmd);
    }

    if (cloudEmergencyStop)
    {
        Serial.println(F("!! CLOUD EMERGENCY STOP !!"));
        cloudEmergencyStop = false;
        car.Stop();
        restoreIR();
        return true;
    }

    // Physical IR star button emergency stop.
    if (IrReceiver.decode())
    {
        if (IrReceiver.decodedIRData.command == CMD_STAR)
        {
            Serial.println(F("!! IR EMERGENCY STOP !!"));
            car.Stop();
            restoreIR();
            return true;
        }

        IrReceiver.resume();
    }

    return false;
}

unsigned long readColorPulse(bool s2, bool s3)
{
    digitalWrite(COLOR_S2, s2);
    digitalWrite(COLOR_S3, s3);
    delay(50);

    unsigned long pulse = pulseIn(COLOR_OUT, LOW, 100000);

    if (pulse == 0)
        pulse = 99999;

    return pulse;
}

String detectColorName()
{
    unsigned long red   = readColorPulse(LOW, LOW);
    unsigned long blue  = readColorPulse(LOW, HIGH);
    unsigned long green = readColorPulse(HIGH, HIGH);

    Serial.print("R=");
    Serial.print(red);
    Serial.print(" G=");
    Serial.print(green);
    Serial.print(" B=");
    Serial.println(blue);

    if (red > 60 && green > 60 && blue > 60)
        return "NO OBJECT";

    if (blue < red && blue < green)
        return "BLUE";

    if (red < blue &&
        green < blue &&
        abs((int)red - (int)green) < 25)
        return "YELLOW";

    if (red < green && red < blue)
        return "RED";

    return "UNKNOWN";
}
bool moveBackwardBlocks(int targetBlocks, uint8_t speed);
bool rotateLeft90();
bool rotateRight90();
bool rotateRight180();

bool reverseForTime(unsigned long durationMs, uint8_t speed)
{
    Serial.println(F("--- Reverse By Time ---"));

    setMotorSpeed(speed);

    unsigned long start = millis();

    while (millis() - start < durationMs)
    {
        if (checkEStop())
        {
            car.Stop();
            return false;
        }

        car.Back();
    }

    car.Stop();
    delay(200);

    restoreIR();

    Serial.println(F("--- Reverse Complete ---"));

    return true;
}

bool DetectBlueAndGrab(int targetBlocks, uint8_t startSpeed)
{
  int junctionCount = 0;
  bool onJunction = true;

  openGripper();

  while (junctionCount < targetBlocks)
  {
    if (checkEStop()) return false;

    long distance = readUltrasonicDistance();

    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(F(" cm"));

    // Object is close enough: stop first, then check color
    if (distance > 0 && distance <= OBJECT_STOP_DISTANCE_CM)
    {
      car.Stop();
      delay(400);

      Serial.println(F("Object reached. Checking color..."));

      while (true)
      {
        if (checkEStop()) return false;

        long confirmDistance = readUltrasonicDistance();

        if (confirmDistance <= 0 ||
            confirmDistance > OBJECT_STOP_DISTANCE_CM + 2)
        {
          Serial.println(F("Object removed. Waiting..."));
          car.Stop();
          delay(300);
          continue;
        }

        String color = detectColorName();

        Serial.print(F("Detected Color: "));
        Serial.println(color);

        if (color == "BLUE")
        {
          Serial.println(F("BLUE confirmed. Grab object."));

          car.Stop();
          delay(300);

          closeGripper();
          delay(500);

          restoreIR();
          return true;
        }

        Serial.println(F("Not BLUE. Do not grab."));

        car.Stop();
        openGripper();
        restoreIR();

        return false;
      }
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    uint8_t currentSpeed;

    // Slow down when approaching an object
    if (distance > 0 && distance <= OBJECT_SLOW_DISTANCE_CM)
    {
      currentSpeed = OBJECT_APPROACH_SPEED;

      Serial.println(F("Object nearby. Slow approach mode."));
    }
    else
    {
      int calculatedSpeed = startSpeed - (junctionCount * 7);

      if (calculatedSpeed < SPEED_MIN)
        calculatedSpeed = SPEED_MIN;

      currentSpeed = (uint8_t)calculatedSpeed;
    }

    if (L == HIGH && M == HIGH && R == HIGH)
    {
      if (!onJunction)
      {
        junctionCount++;
        onJunction = true;

        Serial.print(F("Junction: "));
        Serial.println(junctionCount);

        if (junctionCount >= targetBlocks)
          break;
      }

      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }

    delay(60);
  }

  car.Stop();
  Serial.println(F("Reached target junctions without grabbing BLUE."));
  restoreIR();

  return false;
}

bool DetectRedAndGrab(int targetBlocks, uint8_t startSpeed)
{
  int junctionCount = 0;
  bool onJunction = true;

  openGripper();

  while (junctionCount < targetBlocks)
  {
    if (checkEStop()) return false;

    long distance = readUltrasonicDistance();

    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(F(" cm"));

    // Object is close enough: stop first, then check color
    if (distance > 0 && distance <= OBJECT_STOP_DISTANCE_CM)
    {
      car.Stop();
      delay(400);

      Serial.println(F("Object reached. Checking color..."));

      while (true)
      {
        if (checkEStop()) return false;

        long confirmDistance = readUltrasonicDistance();

        if (confirmDistance <= 0 ||
            confirmDistance > OBJECT_STOP_DISTANCE_CM + 2)
        {
          Serial.println(F("Object removed. Waiting..."));
          car.Stop();
          delay(300);
          continue;
        }

        String color = detectColorName();

        Serial.print(F("Detected Color: "));
        Serial.println(color);

        if (color == "RED")
        {
          Serial.println(F("RED confirmed. Grab object."));

          car.Stop();
          delay(300);

          closeGripper();
          delay(500);

          restoreIR();
          return true;
        }

        Serial.println(F("Not RED. Do not grab."));

        car.Stop();
        openGripper();
        restoreIR();

        return false;
      }
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    uint8_t currentSpeed;

    // Slow down when approaching an object
    if (distance > 0 && distance <= OBJECT_SLOW_DISTANCE_CM)
    {
      currentSpeed = OBJECT_APPROACH_SPEED;

      Serial.println(F("Object nearby. Slow approach mode."));
    }
    else
    {
      int calculatedSpeed = startSpeed - (junctionCount * 7);

      if (calculatedSpeed < SPEED_MIN)
        calculatedSpeed = SPEED_MIN;

      currentSpeed = (uint8_t)calculatedSpeed;
    }

    if (L == HIGH && M == HIGH && R == HIGH)
    {
      if (!onJunction)
      {
        junctionCount++;
        onJunction = true;

        Serial.print(F("Junction: "));
        Serial.println(junctionCount);

        if (junctionCount >= targetBlocks)
          break;
      }

      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }

    delay(60);
  }

  car.Stop();
  Serial.println(F("Reached target junctions without grabbing RED."));
  restoreIR();

  return false;
}


bool DetectYellowAndGrab(int targetBlocks, uint8_t startSpeed)
{
  int junctionCount = 0;
  bool onJunction = true;

  openGripper();

  while (junctionCount < targetBlocks)
  {
    if (checkEStop()) return false;

    long distance = readUltrasonicDistance();

    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(F(" cm"));

    // Object is close enough: stop first, then check color
    if (distance > 0 && distance <= OBJECT_STOP_DISTANCE_CM)
    {
      car.Stop();
      delay(400);

      Serial.println(F("Object reached. Checking color..."));

      while (true)
      {
        if (checkEStop()) return false;

        long confirmDistance = readUltrasonicDistance();

        if (confirmDistance <= 0 ||
            confirmDistance > OBJECT_STOP_DISTANCE_CM + 2)
        {
          Serial.println(F("Object removed. Waiting..."));
          car.Stop();
          delay(300);
          continue;
        }

        String color = detectColorName();

        Serial.print(F("Detected Color: "));
        Serial.println(color);

        if (color == "YELLOW")
        {
          Serial.println(F("YELLOW confirmed. Grab object."));

          car.Stop();
          delay(300);

          closeGripper();
          delay(500);

          restoreIR();
          return true;
        }

        Serial.println(F("Not YELLOW. Do not grab."));

        car.Stop();
        openGripper();
        restoreIR();

        return false;
      }
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    uint8_t currentSpeed;

    // Slow down when approaching an object
    if (distance > 0 && distance <= OBJECT_SLOW_DISTANCE_CM)
    {
      currentSpeed = OBJECT_APPROACH_SPEED;

      Serial.println(F("Object nearby. Slow approach mode."));
    }
    else
    {
      int calculatedSpeed = startSpeed - (junctionCount * 7);

      if (calculatedSpeed < SPEED_MIN)
        calculatedSpeed = SPEED_MIN;

      currentSpeed = (uint8_t)calculatedSpeed;
    }

    if (L == HIGH && M == HIGH && R == HIGH)
    {
      if (!onJunction)
      {
        junctionCount++;
        onJunction = true;

        Serial.print(F("Junction: "));
        Serial.println(junctionCount);

        if (junctionCount >= targetBlocks)
          break;
      }

      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == LOW && M == LOW && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }
    else if (L == HIGH && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == LOW && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Advance();
    }
    else if (L == HIGH && M == HIGH && R == LOW)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Left();
    }
    else if (L == LOW && M == HIGH && R == HIGH)
    {
      onJunction = false;
      setMotorSpeed(currentSpeed);
      car.Turn_Right();
    }

    delay(60);
  }

  car.Stop();
  Serial.println(F("Reached target junctions without grabbing YELLOW."));
  restoreIR();

  return false;
}


// ================================================================
//  MOVEMENT CONTROLS
// ================================================================
bool DetectAndGrab(int targetBlocks, uint8_t startSpeed) {
  int junctionCount = 0;
  bool onJunction = true;
  openGripper();

  while (junctionCount < targetBlocks) {
    if (checkEStop()) return false;

    long distance = readUltrasonicDistance();
    if (distance > 0 && distance <= GRAB_DISTANCE_CM) {
      car.Stop();
      closeGripper();
      restoreIR();
      return true;
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    int calculatedSpeed = startSpeed - (junctionCount * 7);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    if (L == HIGH && M == HIGH && R == HIGH) {
      if (!onJunction) {
        junctionCount++;
        onJunction = true;
        if (junctionCount == targetBlocks) break;
      }
      setMotorSpeed(currentSpeed); car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)  { onJunction = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == LOW && R == HIGH)  { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
    else if (L == HIGH && M == LOW && R == LOW)  { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == LOW && R == LOW)   { onJunction = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == HIGH && M == HIGH && R == LOW) { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == HIGH && R == HIGH) { onJunction = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
    delay(10);
  }
  car.Stop();
  restoreIR();
  return true;
}

// ================================================================
//  TURN 90 DEGREES RIGHT (Maintains SPEED_ROTATE)
// ================================================================
bool rotateRight90() {
  Serial.println(F("\n--- Rotating 90 Degrees Right ---"));

  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEStop()) return false;
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();
  }

  while (true) {
    if (checkEStop()) return false;

    if (millis() - t_start >= ROTATE_TIMEOUT_MS) {
      Serial.println(F("Rotate right timeout! Proceeding to next step."));
      break;
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
//  TURN 90 DEGREES LEFT (Maintains SPEED_ROTATE)
// ================================================================
bool rotateLeft90() {
  Serial.println(F("\n--- Rotating 90 Degrees Left ---"));

  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEStop()) return false;
    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();
  }

  while (true) {
    if (checkEStop()) return false;
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
//  DRIFT LEFT UNTIL LINE (Rear wheels only, pivots on front axle)
// ================================================================
bool driftLeftToLine(uint8_t speed, unsigned long timeoutMs = 0) {
  Serial.println(F("\n--- Drifting Left Until Line ---"));

  unsigned long t_start = millis();
  while (millis() - t_start < TURN_BLIND_MS) {
    if (checkEStop()) return false;
    setMotorSpeed(speed);
    car.drift_left();
  }

  while (true) {
    if (checkEStop()) return false;

    if (timeoutMs > 0 && millis() - t_start >= timeoutMs) {
      Serial.println(F("Drift left timeout! Proceeding to next step."));
      break;
    }

    setMotorSpeed(speed);
    car.drift_left();

    if (digitalRead(STOP_SENSOR_LEFT_TURN) == HIGH) {
      Serial.println(F("Line found! Stopping drift."));
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

bool moveLineTracking(ActionType type, int targetBlocks, uint8_t startSpeed, unsigned long timeoutMs = 0) {
  int count = 0;
  bool onMark = true;
  unsigned long t_start = millis();
  moveTimedOut = false;

  while (count < targetBlocks) {
    if (checkEStop()) return false;

    if (timeoutMs > 0 && millis() - t_start >= timeoutMs) {
      Serial.println(F("Line tracking timeout! Proceeding to next step."));
      moveTimedOut = true;
      car.Stop();
      restoreIR();
      return true;
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    int decelStep = (type == MOVE_FORWARD) ? 7 : 3;
    int calculatedSpeed = startSpeed - (count * decelStep);
    if (calculatedSpeed < SPEED_MIN) calculatedSpeed = SPEED_MIN;
    uint8_t currentSpeed = (uint8_t)calculatedSpeed;

    bool triggered = false;
    if (type == MOVE_FORWARD)       triggered = (L == HIGH && M == HIGH && R == HIGH);
    else if (type == MOVE_LEFT_MARKERS)  triggered = (M == HIGH && L == HIGH);
    else if (type == MOVE_RIGHT_MARKERS) triggered = (M == HIGH && R == HIGH);

    if (triggered) {
      if (!onMark) {
        count++;
        onMark = true;

        Serial.print(F("Marker: "));
        Serial.print(count);
        Serial.print(F(" / "));
        Serial.println(targetBlocks);

        if (count == targetBlocks) break;
      }
      setMotorSpeed(currentSpeed); car.Advance();
    }
    else if (L == LOW && M == HIGH && R == LOW)  { onMark = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == LOW && M == LOW && R == HIGH)  { onMark = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
    else if (L == HIGH && M == LOW && R == LOW)  { onMark = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == LOW && R == LOW)   { onMark = false; setMotorSpeed(currentSpeed); car.Advance(); }
    else if (L == HIGH && M == HIGH && R == LOW) { onMark = false; setMotorSpeed(currentSpeed + 10); car.Turn_Left(); }
    else if (L == LOW && M == HIGH && R == HIGH) { onMark = false; setMotorSpeed(currentSpeed + 10); car.Turn_Right(); }
  }

  if (count == targetBlocks && CENTER_OFFSET_MS > 0) {
    setMotorSpeed(SPEED_MIN); car.Advance(); delay(CENTER_OFFSET_MS);
  }
  restoreIR();
  return true;
}

bool moveBackwardBlocks(int targetBlocks, uint8_t speed) {
  int junctionCount = 0;
  bool onJunction = true;

  const uint8_t correction = 7;
  const uint8_t minimumSideSpeed = 28;

  Serial.print(F("\n--- Reverse "));
  Serial.print(targetBlocks);
  Serial.println(F(" Junctions ---"));

  while (junctionCount < targetBlocks) {
    if (checkEStop()) {
      car.Stop();
      return false;
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    uint8_t slowerSpeed =
      (speed > correction) ? speed - correction : minimumSideSpeed;

    uint8_t fasterSpeed = speed + correction;

    // Full junction
    if (L == HIGH && M == HIGH && R == HIGH) {
      if (!onJunction) {
        junctionCount++;
        onJunction = true;

        Serial.print(F("Reverse Junction: "));
        Serial.println(junctionCount);

        if (junctionCount >= targetBlocks) {
          break;
        }
      }

      setMotorSpeedLR(speed, speed);
      car.Back();
    }

    // Properly centered
    else if (L == LOW && M == HIGH && R == LOW) {
      onJunction = false;

      setMotorSpeedLR(speed, speed);
      car.Back();
    }

    // Line detected on right
    // Gently correct left while continuing backward
    else if (L == LOW && M == LOW && R == HIGH) {
      onJunction = false;

      setMotorSpeedLR(fasterSpeed, slowerSpeed);
      car.Back();
    }

    // Line detected on left
    // Gently correct right while continuing backward
    else if (L == HIGH && M == LOW && R == LOW) {
      onJunction = false;

      setMotorSpeedLR(slowerSpeed, fasterSpeed);
      car.Back();
    }

    // No line: keep reversing straight slowly
    else if (L == LOW && M == LOW && R == LOW) {
      onJunction = false;

      setMotorSpeedLR(speed, speed);
      car.Back();
    }

    // Left + middle
    else if (L == HIGH && M == HIGH && R == LOW) {
      onJunction = false;

      setMotorSpeedLR(slowerSpeed, fasterSpeed);
      car.Back();
    }

    // Middle + right
    else if (L == LOW && M == HIGH && R == HIGH) {
      onJunction = false;

      setMotorSpeedLR(fasterSpeed, slowerSpeed);
      car.Back();
    }

    delay(15);
  }

  // Move backward slightly to leave the junction
  setMotorSpeedLR(speed, speed);
  car.Back();
  delay(100);

  car.Stop();
  delay(300);

  restoreIR();

  Serial.println(F("--- Reverse Complete and Aligned ---"));
  return true;
}

// ================================================================
//  SEQUENCE EXECUTION MOTOR
// ================================================================
void executeSequence(const Step sequence[], int totalSteps) {
  for (int i = 0; i < totalSteps; i++) {
    bool stepSuccess = true;

    // Sequence tables live in flash (PROGMEM); copy one step into RAM
    Step step;
    memcpy_P(&step, &sequence[i], sizeof(Step));

    switch (step.action) {
      case MOVE_FORWARD:
      case MOVE_LEFT_MARKERS:
      case MOVE_RIGHT_MARKERS:
        stepSuccess = moveLineTracking(step.action, step.param1, step.param2);
        break;
      case MOVE_FORWARD_TIMED:
        stepSuccess = moveLineTracking(
          MOVE_FORWARD,
          step.param1,
          step.param2,
          MOVE_FWD_TIMEOUT_MS
        );
        break;
        case MOVE_BACKWARD:
        stepSuccess = moveBackwardBlocks(
          step.param1,
          step.param2
        );
        break;
      case ROTATE_LEFT:
        stepSuccess = rotateLeft90();
        break;
      case ROTATE_RIGHT:
        stepSuccess = rotateRight90();
        break;
      case ROTATE_180:
        stepSuccess = rotateRight180();
        break;
      case DRIFT_LEFT:
        stepSuccess = driftLeftToLine(step.param2);
        break;
      case DRIFT_LEFT_TIMED:
        stepSuccess = driftLeftToLine(step.param2, DRIFT_LEFT_TIMEOUT_MS);
        break;
      case DETECT_GRAB:
        stepSuccess = DetectAndGrab(step.param1, step.param2);
        break;
        case DETECT_BLUE_GRAB:
        stepSuccess = DetectBlueAndGrab(
            step.param1,
            step.param2
        );
        break;
      case DETECT_RED_GRAB:
        stepSuccess = DetectRedAndGrab(
            step.param1,
            step.param2
        );
        break;
      case DETECT_YELLOW_GRAB:
        stepSuccess = DetectYellowAndGrab(
            step.param1,
            step.param2
        );
        break;
      case OPEN_GRIPPER:
        openGripper();
        break;
      case DELAY_MS:
        delay(step.param1);
        break;
      case QUICK_REVERSE:
        setMotorSpeed(step.param2); // Uses assigned speed parameter
        car.Back();
        delay(step.param1);         // Backs up for assigned duration parameter
        car.Stop();
        break;
        case REVERSE_TIME:
        stepSuccess = reverseForTime(
        step.param1,   // milliseconds
        step.param2    // speed
    );
    break;

      case REVERSE_IF_TIMEOUT:
        // Runs only when the preceding timed move gave up mid-course
        // (robot may be pressed against something); otherwise a no-op.
        if (moveTimedOut) {
          stepSuccess = reverseForTime(step.param1, step.param2);
        }
        break;

      case STOP_WHEELS:
        car.Stop();
        break;

    }
    
    if (!stepSuccess) {
      Serial.println(F("Sequence Termination triggered."));
      break; 
    }
  }
  car.Stop();
}

void runBlueSuccessPath1() {
  static const Step successPath1Deliver[] PROGMEM = {
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {DRIFT_LEFT_TIMED, 0, SPEED_MIN},
    {DELAY_MS, 1000, 0},

    {MOVE_FORWARD_TIMED, 7, SPEED_START_SLOW},
    {STOP_WHEELS, 0, 0},
    {DELAY_MS, 500, 0},

    {REVERSE_IF_TIMEOUT, 100, 35},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 1000, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},
  };

  static const Step successPath1Return[] PROGMEM = {
    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath1Deliver,
    sizeof(successPath1Deliver) / sizeof(successPath1Deliver[0])
  );

  sendStatusToFavoriot("task4_complete");

  executeSequence(
    successPath1Return,
    sizeof(successPath1Return) / sizeof(successPath1Return[0])
  );
}

void runBlueSuccessPath2() {
  static const Step successPath2Deliver[] PROGMEM = {
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {STOP_WHEELS, 0, 0},
    {DELAY_MS, 500, 0},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},
  };

  static const Step successPath2Return[] PROGMEM = {
    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath2Deliver,
    sizeof(successPath2Deliver) / sizeof(successPath2Deliver[0])
  );

  sendStatusToFavoriot("task4_complete");

  executeSequence(
    successPath2Return,
    sizeof(successPath2Return) / sizeof(successPath2Return[0])
  );
}

void runBlueSuccessPath3() {
  static const Step successPath3Deliver[] PROGMEM = {
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {STOP_WHEELS, 0, 0},
    {DELAY_MS, 500, 0},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},
    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},
  };

  static const Step successPath3Return[] PROGMEM = {
    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath3Deliver,
    sizeof(successPath3Deliver) / sizeof(successPath3Deliver[0])
  );

  sendStatusToFavoriot("task4_complete");

  executeSequence(
    successPath3Return,
    sizeof(successPath3Return) / sizeof(successPath3Return[0])
  );
}

// Go for Left Side
void goFromMiddleToLeftSide() {
  static const Step goToLeftSide[] PROGMEM = {
    {REVERSE_TIME, 1500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    goToLeftSide,
    sizeof(goToLeftSide) / sizeof(goToLeftSide[0])
  );
}

void goFromLeftToRightSide() {
  static const Step goToRightSide[] PROGMEM = {
    {REVERSE_TIME, 1500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 4, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    goToRightSide,
    sizeof(goToRightSide) / sizeof(goToRightSide[0])
  );
}

bool runCmd2MiddlePath()
{
    static const Step middlePath[] PROGMEM = {
        {DETECT_GRAB, 6, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_FORWARD, 6, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {OPEN_GRIPPER, 0, 0},
        {DELAY_MS, 300, 0},

        {REVERSE_TIME, 500, 35},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
        {DELAY_MS, 500, 0}
    };

    executeSequence(
        middlePath,
        sizeof(middlePath) / sizeof(middlePath[0])
    );

    return true;
}

bool runCmd2LeftPath()
{
    static const Step leftPath[] PROGMEM = {
        {MOVE_FORWARD, 3, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_LEFT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {DETECT_GRAB, 3, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_FORWARD, 7, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {OPEN_GRIPPER, 0, 0},
        {DELAY_MS, 300, 0},

        {REVERSE_TIME, 500, 35},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
        {DELAY_MS, 500, 0}
    };

    executeSequence(
        leftPath,
        sizeof(leftPath) / sizeof(leftPath[0])
    );

    return true;
}

bool runCmd2RightPath()
{
    static const Step rightPath[] PROGMEM = {
        {MOVE_FORWARD, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_LEFT, 0, 0},
        {DELAY_MS, 500, 0},

        {DETECT_GRAB, 4, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_LEFT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {ROTATE_LEFT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_FORWARD, 7, SPEED_START_SLOW},
        {DELAY_MS, 500, 0},

        {OPEN_GRIPPER, 0, 0},
        {DELAY_MS, 300, 0},

        {REVERSE_TIME, 500, 35},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {ROTATE_RIGHT, 0, 0},
        {DELAY_MS, 500, 0},

        {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
        {DELAY_MS, 500, 0}
    };

    executeSequence(
        rightPath,
        sizeof(rightPath) / sizeof(rightPath[0])
    );

    return true;
}

void runRemote2Mission()
{
    Serial.println(F("Starting Remote 2 mission."));

    runCmd2MiddlePath();
    runCmd2LeftPath();
    runCmd2RightPath();

    car.Stop();
    restoreIR();

    Serial.println(F("Remote 2 mission complete."));
}

void runRemote4Mission()
{
  Serial.println(F("Starting Remote 4 mission."));

  // ============================================================
  // POSITION 1: MIDDLE
  // ============================================================
  // static const Step approachMiddle[] PROGMEM = {
  //   {MOVE_FORWARD, 3, SPEED_START_SLOW},
  //   {DELAY_MS, 500, 0}
  // };

  // executeSequence(
  //   approachMiddle,
  //   sizeof(approachMiddle) / sizeof(approachMiddle[0])
  // );

  bool firstBlueGrabbed = DetectBlueAndGrab(
    7,
    SPEED_START_SLOW
  );

  if (firstBlueGrabbed) {
    Serial.println(F("Blue found at middle position."));
    runBlueSuccessPath1();
    return;
  }

  // ============================================================
  // POSITION 2: LEFT SIDE
  // ============================================================
  Serial.println(F("Middle object is not blue. Going left."));

  goFromMiddleToLeftSide();

  bool secondBlueGrabbed = DetectBlueAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (secondBlueGrabbed) {
    Serial.println(F("Blue found at left position."));
    runBlueSuccessPath2();
    return;
  }

  // ============================================================
  // POSITION 3: RIGHT SIDE
  // ============================================================
  Serial.println(F("Left object is not blue. Going right."));

  goFromLeftToRightSide();

  bool thirdBlueGrabbed = DetectBlueAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (thirdBlueGrabbed) {
    Serial.println(F("Blue found at right position."));
    runBlueSuccessPath3();
    return;
  }

  Serial.println(F("No blue object found in all 3 positions."));
  car.Stop();
  restoreIR();
}
// Red (mission 5) and yellow (mission 6) share these delivery routes;
// only the Favoriot status string differs.
void runRedSuccessPath1(const char* status = "task5_complete") {
  static const Step successPath1Deliver[] PROGMEM = {
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {DRIFT_LEFT_TIMED, 0, SPEED_MIN},
    {DELAY_MS, 1000, 0},

    {MOVE_FORWARD_TIMED, 7, SPEED_START_SLOW},
    {STOP_WHEELS, 0, 0},
    {DELAY_MS, 500, 0},

    {REVERSE_IF_TIMEOUT, 100, 35},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 1000, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},
  };

  static const Step successPath1Return[] PROGMEM = {
    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath1Deliver,
    sizeof(successPath1Deliver) / sizeof(successPath1Deliver[0])
  );

  sendStatusToFavoriot(status);

  executeSequence(
    successPath1Return,
    sizeof(successPath1Return) / sizeof(successPath1Return[0])
  );
}

void runRedSuccessPath2(const char* status = "task5_complete") {
  static const Step successPath2Deliver[] PROGMEM = {
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {STOP_WHEELS, 0, 0},
    {DELAY_MS, 500, 0},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},
  };

  static const Step successPath2Return[] PROGMEM = {
    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath2Deliver,
    sizeof(successPath2Deliver) / sizeof(successPath2Deliver[0])
  );

  sendStatusToFavoriot(status);

  executeSequence(
    successPath2Return,
    sizeof(successPath2Return) / sizeof(successPath2Return[0])
  );
}

void runRedSuccessPath3(const char* status = "task5_complete") {
  static const Step successPath3Deliver[] PROGMEM = {
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_FORWARD, 7, SPEED_START_SLOW},
    {STOP_WHEELS, 0, 0},
    {DELAY_MS, 500, 0},

    {OPEN_GRIPPER, 0, 0},
    {DELAY_MS, 300, 0},

    {REVERSE_TIME, 500, 35},
    {DELAY_MS, 500, 0},
  };

  static const Step successPath3Return[] PROGMEM = {
    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 1, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},
  };

  executeSequence(
    successPath3Deliver,
    sizeof(successPath3Deliver) / sizeof(successPath3Deliver[0])
  );

  sendStatusToFavoriot(status);

  executeSequence(
    successPath3Return,
    sizeof(successPath3Return) / sizeof(successPath3Return[0])
  );
}

// Yellow (mission 6) uses the same routes as red; only the status differs.
void runYellowSuccessPath1() { runRedSuccessPath1("task6_complete"); }
void runYellowSuccessPath2() { runRedSuccessPath2("task6_complete"); }
void runYellowSuccessPath3() { runRedSuccessPath3("task6_complete"); }

// Go for Left Side
void goFromMiddleToLeftSideRed() {
  static const Step goToLeftSide[] PROGMEM = {
    {REVERSE_TIME, 1500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    goToLeftSide,
    sizeof(goToLeftSide) / sizeof(goToLeftSide[0])
  );
}

void goFromLeftToRightSideRed() {
  static const Step goToRightSide[] PROGMEM = {
    {REVERSE_TIME, 1500, 35},
    {DELAY_MS, 500, 0},

    {ROTATE_RIGHT, 0, 0},
    {DELAY_MS, 500, 0},

    {MOVE_RIGHT_MARKERS, 4, SPEED_START_SLOW},
    {DELAY_MS, 500, 0},

    {ROTATE_LEFT, 0, 0},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    goToRightSide,
    sizeof(goToRightSide) / sizeof(goToRightSide[0])
  );
}

void runRemote5Mission()
{
  Serial.println(F("Starting Remote 5 mission."));

  // ============================================================
  // POSITION 1: MIDDLE
  // ============================================================
  static const Step approachMiddle[] PROGMEM = {
    {MOVE_FORWARD, 3, SPEED_START_SLOW},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    approachMiddle,
    sizeof(approachMiddle) / sizeof(approachMiddle[0])
  );

  bool firstRedGrabbed = DetectRedAndGrab(
    3,
    SPEED_START_SLOW
  );

  if (firstRedGrabbed) {
    Serial.println(F("Red found at middle position."));
    runRedSuccessPath1();
    return;
  }

  // ============================================================
  // POSITION 2: LEFT SIDE
  // ============================================================
  Serial.println(F("Middle object is not red. Going left."));

  goFromMiddleToLeftSideRed();

  bool secondRedGrabbed = DetectRedAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (secondRedGrabbed) {
    Serial.println(F("Red found at left position."));
    runRedSuccessPath2();
    return;
  }

  // ============================================================
  // POSITION 3: RIGHT SIDE
  // ============================================================
  Serial.println(F("Left object is not red. Going right."));

  goFromLeftToRightSideRed();

  bool thirdRedGrabbed = DetectRedAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (thirdRedGrabbed) {
    Serial.println(F("Red found at right position."));
    runRedSuccessPath3();
    return;
  }

  Serial.println(F("No red object found in all 3 positions."));
  car.Stop();
  restoreIR();
}

void runRemote6Mission()
{
  Serial.println(F("Starting Remote 6 mission."));

  // ============================================================
  // POSITION 1: MIDDLE
  // ============================================================
  static const Step approachMiddle[] PROGMEM = {
    {MOVE_FORWARD, 3, SPEED_START_SLOW},
    {DELAY_MS, 500, 0}
  };

  executeSequence(
    approachMiddle,
    sizeof(approachMiddle) / sizeof(approachMiddle[0])
  );

  bool firstYellowGrabbed = DetectYellowAndGrab(
    3,
    SPEED_START_SLOW
  );

  if (firstYellowGrabbed) {
    Serial.println(F("Yellow found at middle position."));
    runYellowSuccessPath1();
    return;
  }

  // ============================================================
  // POSITION 2: LEFT SIDE
  // ============================================================
  Serial.println(F("Middle object is not yellow. Going left."));

  goFromMiddleToLeftSideRed();

  bool secondYellowGrabbed = DetectYellowAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (secondYellowGrabbed) {
    Serial.println(F("Yellow found at left position."));
    runYellowSuccessPath2();
    return;
  }

  // ============================================================
  // POSITION 3: RIGHT SIDE
  // ============================================================
  Serial.println(F("Left object is not yellow. Going right."));

  goFromLeftToRightSideRed();

  bool thirdYellowGrabbed = DetectYellowAndGrab(
    2,
    SPEED_START_SLOW
  );

  if (thirdYellowGrabbed) {
    Serial.println(F("Yellow found at right position."));
    runYellowSuccessPath3();
    return;
  }

  Serial.println(F("No yellow object found in all 3 positions."));
  car.Stop();
  restoreIR();
}

void readCloudCommand()
{
    if (!espSerial.available())
        return;

    String cmd = espSerial.readStringUntil('\n');

    cmd.trim();
    cmd.toUpperCase();

    Serial.print(F("Cloud Command: "));
    Serial.println(cmd);

    if (cmd == "CMD4")
    {
        Serial.println(F("Cloud Start Remote4"));
        runRemote4Mission();
    }
    else if (cmd == "CMD5")
    {
        Serial.println(F("Cloud Start Remote5"));
        runRemote5Mission();
    }
    else if (cmd == "CMD6")
    {
        Serial.println(F("Cloud Start Remote6"));
        runRemote6Mission();
    }
    else if (
        cmd == "STOP" ||
        cmd == "STAR" ||
        cmd == "CMDSTAR" ||
        cmd == "EMERGENCY_STOP"
    )
    {
        Serial.println(F("!!! CLOUD EMERGENCY STOP !!!"));
        cloudEmergencyStop = false;
        car.Stop();
        restoreIR();
    }
}




// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);
  espSerial.setTimeout(50);

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MID, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);

  gripperServo.attach(GRIPPER_PIN);
  openGripper();

  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  Serial.println(F("Ready."));
  espSerial.println(F("UNO_READY"));
}

void loop()
{
    // Receive cloud commands while the robot is idle.
    readCloudCommand();

    if (!IrReceiver.decode())
        return;

    uint8_t cmd = IrReceiver.decodedIRData.command;

    if (
        cmd == 0x00 ||
        (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)
    )
    {
        IrReceiver.resume();
        return;
    }

    switch (cmd)
    {
        case CMD_1:
        {
            static const Step path1[] PROGMEM = {
                {MOVE_FORWARD, 3, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_LEFT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_RIGHT, 0, 0},
                {DETECT_GRAB, 3, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_RIGHT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_RIGHT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_FORWARD, 7, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {OPEN_GRIPPER, 0, 0},
                {MOVE_BACKWARD, 1, SPEED_START_SLOW}
            };

            executeSequence(
                path1,
                sizeof(path1) / sizeof(path1[0])
            );
            break;
        }

        case CMD_2:
            runRemote2Mission();
            break;
        

        case CMD_3:
        {
            static const Step path3[] PROGMEM = {
                {MOVE_FORWARD, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_RIGHT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_RIGHT_MARKERS, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_LEFT, 0, 0},
                {DETECT_GRAB, 4, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_LEFT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_LEFT_MARKERS, 2, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {ROTATE_LEFT, 0, 0},
                {DELAY_MS, 500, 0},
                {MOVE_FORWARD, 7, SPEED_START_SLOW},
                {DELAY_MS, 500, 0},
                {OPEN_GRIPPER, 0, 0},
                {MOVE_BACKWARD, 1, SPEED_START_SLOW}
            };

            executeSequence(
                path3,
                sizeof(path3) / sizeof(path3[0])
            );
            break;
        }

        case CMD_4:
            runRemote4Mission();
            break;

        case CMD_5:
            runRemote5Mission();
            break;

        case CMD_6:
            runRemote6Mission();
            break;

        case CMD_STAR:
            Serial.println(F("!! EMERGENCY STOP !!"));
            cloudEmergencyStop = false;
            car.Stop();
            restoreIR();
            break;
    }

    IrReceiver.resume();
}

