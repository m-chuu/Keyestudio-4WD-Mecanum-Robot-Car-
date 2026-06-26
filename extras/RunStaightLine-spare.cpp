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
// Rear wheels are currently loose/intermittent, so by default ONLY the two
// front wheels drive + steer. Once the rear mounts are tightened, set this to
// 1 to get full 4-wheel traction (more stable line-tracking).
#define DRIVE_REAR_WHEELS  0

// Forward speeds, per side, from the straight-line calibration.
// Calibration found the RIGHT wheels run stronger (straight at L=80 / R=74,
// ratio ~0.925), so the right side is trimmed below the left. Scaled here to
// the crawl speed used for line-following.
#define LINE_SPEED_L     35  // left  front base (0-255)
#define LINE_SPEED_R     32  // right front base (~ 35 * 74/80)
#define TURN_SLOW_SPEED  15  // slowed inner front wheel during a correction
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

// Drive the two FRONT wheels forward at independent speeds so the robot can
// steer (the slower side is the direction it curves toward). Re-asserted every
// loop so a dropped I2C frame self-corrects within one pass.
void driveFront(uint8_t frontLeftSpd, uint8_t frontRightSpd) {
  car.Motor_Upper_L(1, frontLeftSpd);   // front-left  forward (dir 1 = forward)
  car.Motor_Upper_R(1, frontRightSpd);  // front-right forward

#if DRIVE_REAR_WHEELS
  car.Motor_Lower_L(1, LINE_SPEED_L);   // rear-left  at left-side trim
  car.Motor_Lower_R(1, LINE_SPEED_R);   // rear-right at right-side trim
#else
  car.Motor_Lower_L(0, 0);              // rears off (loose-wheel workaround)
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

  // Must begin on the line, otherwise there is nothing to follow.
  if (!ON_BLACK(SENSOR_MID)) {
    Serial.println(F("  ! Middle sensor is NOT on black. Place the robot on the line first."));
    restoreIR();
    return;
  }

  while (true) {
    bool onLeft  = ON_BLACK(SENSOR_LEFT);
    bool onMid   = ON_BLACK(SENSOR_MID);
    bool onRight = ON_BLACK(SENSOR_RIGHT);

    if (!onLeft && !onMid && !onRight) {        // line lost completely
      Serial.println(F("  Line lost - stopping."));
      break;
    }

    if (onRight) {
      // drifted left -> steer RIGHT (slow the front-right wheel)
      driveFront(LINE_SPEED_L, TURN_SLOW_SPEED);
    } else if (onLeft) {
      // drifted right -> steer LEFT (slow the front-left wheel)
      driveFront(TURN_SLOW_SPEED, LINE_SPEED_R);
    } else {
      // only middle sees black -> centered -> straight (calibrated L/R trim)
      driveFront(LINE_SPEED_L, LINE_SPEED_R);
    }

    if (emergencyStopPressed()) {
      Serial.println(F("  * Emergency stop pressed."));
      break;
    }
    delay(5);                                   // steady sensor-polling pace
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

  if (!ON_BLACK(SENSOR_MID)) {
    Serial.println(F("  ! Middle sensor is NOT on black. Place the robot on the line first."));
    restoreIR();
    return;
  }

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
    else if (!onLeft && !onMid && !onRight) {            // nothing -> line lost
      Serial.println(F("  Line lost - stopping."));
      break;
    }
    // else: ambiguous (junction / both outer) -> keep the previous error

    // 2. PID TERMS
    integral += error;
    int derivative = error - lastError;
    lastError = error;

    // 3. CORRECTION (+ steers right: left wheel faster, right slower)
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // 4. APPLY to the calibrated per-side base speeds, clamped to 0..255
    int leftSpd  = constrain((int)(LINE_SPEED_L + correction), 0, 255);
    int rightSpd = constrain((int)(LINE_SPEED_R - correction), 0, 255);
    driveFront(leftSpd, rightSpd);

    if (emergencyStopPressed()) {
      Serial.println(F("  * Emergency stop pressed."));
      break;
    }
    delay(10);   // fixed time step keeps the derivative term consistent
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
