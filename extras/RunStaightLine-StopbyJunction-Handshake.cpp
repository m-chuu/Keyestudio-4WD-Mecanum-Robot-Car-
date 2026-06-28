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
#define CMD_3           0x0D // Press '3' = follow line, stop after N junctions
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

// True only when ALL three sensors sit on black at once — i.e. the car is
// straddling a horizontal cross-line (a junction). Adapted from your snippet
// to this sketch's pin macros (SENSOR_LEFT/MID/RIGHT).
static bool allOnBlack() {
  return ON_BLACK(SENSOR_LEFT) && ON_BLACK(SENSOR_MID) && ON_BLACK(SENSOR_RIGHT);
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
//  FOLLOW LINE, STOP AFTER N JUNCTIONS  (bang-bang steering + counting)
//
//  Crawls forward using the EXACT bang-bang steering from followLine()
//  (outer sensors trim one wheel via TURN_SLOW_SPEED) while watching for
//  horizontal cross-lines. Each time all three sensors hit black at once
//  (allOnBlack) a new junction is counted; the car stops the instant the
//  target count is reached — sitting on that final cross-line.
//
//   • onJunction starts TRUE so the line under the car at launch isn't counted.
//   • It re-arms (onJunction = false) only after the car is back on a plain
//     vertical line (middle sensor only), so one wide cross-line counts once.
//   • Line lost (no sensor on black) -> keep driving straight (per followLine).
//   • '*' on the remote aborts at any time.
// ================================================================
void stopByCounting(int targetCount) {
  Serial.print(F("Counting START - follow line, stop after "));
  Serial.print(targetCount);
  Serial.println(F(" junction(s)."));

  int  count = 0;            // how many cross-lines crossed so far
  bool onJunction = true;    // true while straddling a cross-line (starts true
                             // so the launch line isn't counted)
  unsigned long lastReport = 0;   // non-blocking serial-debug timer

  while (count < targetCount) {
    bool onLeft  = ON_BLACK(SENSOR_LEFT);
    bool onRight = ON_BLACK(SENSOR_RIGHT);

    // --- Non-blocking debug: report sensor states every 500 ms ---
    if (millis() - lastReport > 500) {
      lastReport = millis();
      Serial.print(F("[SEARCH] IR L/C/R = "));
      Serial.print(ON_BLACK(SENSOR_LEFT)  ? 1 : 0); Serial.print(F(" "));
      Serial.print(ON_BLACK(SENSOR_MID)   ? 1 : 0); Serial.print(F(" "));
      Serial.print(ON_BLACK(SENSOR_RIGHT) ? 1 : 0);
      Serial.print(F("   (count="));
      Serial.print(count);
      Serial.println(F(", motors commanded FORWARD)"));
    }

    // --- JUNCTION: all three sensors on black at once ---
    if (allOnBlack()) {
      if (!onJunction) {            // rising edge: a brand-new cross-line
        count++;
        onJunction = true;          // lock so this same line counts only once
        Serial.print(F("  Junction crossed: "));
        Serial.println(count);
        if (count == targetCount) {
          break;                    // target hit -> stop on this line
        }
      }
      // Keep crossing straight so the steering logic doesn't misread the
      // wide black band as a one-sided correction.
      driveFront(LINE_SPEED_L, LINE_SPEED_R);
    }
    // --- NOT a junction: steer to stay on the line (exact followLine logic) ---
    else {
      // Re-arm the counter once back on a plain vertical line (middle only).
      if (!onLeft && !onRight) {
        onJunction = false;
      }

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
    }

    if (emergencyStopPressed()) {
      Serial.println(F("  * Emergency stop pressed."));
      break;
    }
  }

  car.Stop();
  Serial.print(F("Counting STOP - junctions crossed: "));
  Serial.println(count);
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

  Serial.println(F("Ready. '3' = follow + stop after 3 junctions, '*' = stop."));
}

void loop() {
  if (IrReceiver.decode()) {
    uint8_t key = IrReceiver.decodedIRData.command;
    IrReceiver.resume();

    if (key == CMD_3) {
      stopByCounting(3);   // follow line, stop after 3 junctions
    }
  }
  delay(50);
}
