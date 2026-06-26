#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.hpp>

// ── Hardware pins ─────────────────────────────────────────────
#define RECV_PIN        A3
#define SENSOR_LEFT     A0   // left line sensor
#define SENSOR_MID      A1   // middle sensor — should stay on the black line
#define SENSOR_RIGHT    A2   // right line sensor

// ── EXTERN LIBRARY VARIABLES ──────────────────────────────────
extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

// ── IR Codes ──────────────────────────────────────────────────
#define CMD_1           0x16 // Press '1' = bang-bang line-following
#define CMD_2           0x19 // Press '2' = PID line-following
#define CMD_STAR        0x42 // '*' = emergency stop

// ── Drive configuration ───────────────────────────────────────
// 1 = all four wheels drive AND steer (rears mirror the front speeds).
// Set to 0 for front-wheel-only drive (e.g. if a rear wheel is loose).
#define DRIVE_REAR_WHEELS  1

// Forward base speeds, per side, from the straight-line calibration.
// The RIGHT wheels run stronger, so the right base is trimmed below the left
// (straight-line values L=80 / R=74). Used directly as the base speeds.
#define LINE_SPEED_L     80  // left  base (0-255)
#define LINE_SPEED_R     74  // right base (calibrated straight-line value)
#define TURN_SLOW_SPEED  15  // slowed inner wheels during a correction
                             // (smaller = sharper turn; raise toward base = gentler)

// Black-line detection — threshold taken from
// PhysicVector-balckline-sensitive.cpp:
//   white floor ~25, black tape ~750.  Strict: analog > 700 means black.
#define BLACK_THRESHOLD 700
#define ON_BLACK(pin)   (analogRead(pin) > BLACK_THRESHOLD)

mecanumCar car(3, 2);

// ================================================================
//  HELPERS
// ================================================================
void restoreIR() {
  car.Stop();
  IrReceiver.begin(RECV_PIN, false);
}

// Returns true if the '*' emergency-stop key was pressed.
bool emergencyStopPressed() {
  if (IrReceiver.decode()) {
    bool stop = (IrReceiver.decodedIRData.command == CMD_STAR);
    IrReceiver.resume();           // ready for the next code
    return stop;
  }
  return false;
}

// Drive the LEFT and RIGHT wheels forward at independent speeds so the robot
// can steer (the slower side is the direction it curves toward). With 4WD on,
// the rear wheels MIRROR the front speeds so all four wheels steer together.
// Re-asserted every loop so a dropped I2C frame self-corrects within one pass.
void driveFront(uint8_t leftSpd, uint8_t rightSpd) {
  car.Motor_Upper_L(1, leftSpd);    // front-left  forward (dir 1 = forward)
  car.Motor_Upper_R(1, rightSpd);   // front-right forward

#if DRIVE_REAR_WHEELS
  car.Motor_Lower_L(1, leftSpd);    // rear-left  mirrors front-left  (steers too)
  car.Motor_Lower_R(1, rightSpd);   // rear-right mirrors front-right (steers too)
#else
  car.Motor_Lower_L(0, 0);          // rears off (front-wheel-only mode)
  car.Motor_Lower_R(0, 0);
#endif
}

// ================================================================
//  LINE FOLLOW  (steering correction on the two front wheels)
//
//  Crawl forward while keeping the MIDDLE sensor on the black line.
//   • line drifts under RIGHT sensor  -> body moved LEFT  -> steer RIGHT
//        (slow the front-RIGHT wheel so the robot curves right)
//   • line drifts under LEFT  sensor  -> body moved RIGHT -> steer LEFT
//        (slow the front-LEFT wheel so the robot curves left)
//   • only MIDDLE on black            -> centered -> drive straight
//   • no sensor on black              -> line lost -> STOP
//  '*' on the remote stops at any time.
// ================================================================
void followLine() {
  Serial.println(F("Line-follow START - crawling forward with steering correction."));

  while (true) {
    bool onLeft  = ON_BLACK(SENSOR_LEFT);
    bool onRight = ON_BLACK(SENSOR_RIGHT);

    if (onRight) {
      // drifted left -> steer RIGHT (slow the right wheels)
      driveFront(LINE_SPEED_L, TURN_SLOW_SPEED);
    } else if (onLeft) {
      // drifted right -> steer LEFT (slow the left wheels)
      driveFront(TURN_SLOW_SPEED, LINE_SPEED_R);
    } else {
      // centered OR line lost -> drive straight at base speeds
      driveFront(LINE_SPEED_L, LINE_SPEED_R);
    }

    if (emergencyStopPressed()) {
      Serial.println(F("  * Emergency stop pressed."));
      break;
    }
  }

  car.Stop();
  Serial.println(F("Line-follow STOP."));
  restoreIR();
}

// ================================================================
//  LINE FOLLOW  (PID version — alternative to the bang-bang one above)
//
//  Same sensor/steering convention, but instead of three fixed speeds
//  it builds a continuous correction from a 5-level error (-2..+2) and
//  trims the two front wheels proportionally.
//
//  With only 3 on/off sensors the error is coarse, so:
//    Kp (proportional) does the real work — graduated steering
//    Kd (derivative)   damps wobble at transitions
//    Ki (integral)     keep 0 — near-useless on 3 sensors, prone to windup
// ================================================================
float Kp = 12.0;   // steering strength  (lower if it oscillates)
float Kd = 5.0;    // wobble damping
float Ki = 0.0;    // leave at 0

void followLinePID() {
  Serial.println(F("Line-follow START - PID control active."));

  // Fresh state each run (must NOT carry over from a previous run).
  int  error = 0, lastError = 0;
  long integral = 0;

  while (true) {
    bool onLeft  = ON_BLACK(SENSOR_LEFT);
    bool onMid   = ON_BLACK(SENSOR_MID);
    bool onRight = ON_BLACK(SENSOR_RIGHT);

    // 1. ERROR: where is the line vs centre? (+ = robot is LEFT of the line)
    if      ( onLeft && !onMid && !onRight) error = -2;  // robot far right
    else if ( onLeft &&  onMid && !onRight) error = -1;  // robot slightly right
    else if (!onLeft &&  onMid && !onRight) error =  0;  // centred
    else if (!onLeft &&  onMid &&  onRight) error =  1;  // robot slightly left
    else if (!onLeft && !onMid &&  onRight) error =  2;  // robot far left
    else if (!onLeft && !onMid && !onRight) error =  0;  // line lost -> go straight
    // else: ambiguous (junction / both outer) -> keep the previous error

    // 2. PID TERMS
    integral += error;
    int derivative = error - lastError;
    lastError = error;

    // 3. CORRECTION (+ steers right: left wheels faster, right slower)
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // 4. APPLY to the base speeds, clamped to 0..255
    int leftSpd  = constrain((int)(LINE_SPEED_L + correction), 0, 255);
    int rightSpd = constrain((int)(LINE_SPEED_R - correction), 0, 255);
    driveFront(leftSpd, rightSpd);

    if (emergencyStopPressed()) {
      Serial.println(F("  * Emergency stop pressed."));
      break;
    }
  }

  car.Stop();
  Serial.println(F("Line-follow STOP."));
  restoreIR();
}

// ================================================================
//  ARDUINO ENTRY POINTS
// ================================================================
void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }            // wait for serial (needed on native-USB boards)

  car.Init();                      // bring the motor controller to a known state
  IrReceiver.begin(RECV_PIN, false);

  Serial.println(F("Ready. '1' = bang-bang follow, '2' = PID follow, '*' = stop."));
}

void loop() {
  if (IrReceiver.decode()) {
    uint8_t key = IrReceiver.decodedIRData.command;
    IrReceiver.resume();

    if (key == CMD_1) {
      followLine();        // bang-bang (3 fixed speeds)
    } else if (key == CMD_2) {
      followLinePID();     // PID (graduated correction)
    }
  }
  delay(50);
}
