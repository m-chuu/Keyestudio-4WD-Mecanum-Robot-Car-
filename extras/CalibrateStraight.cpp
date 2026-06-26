#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.hpp>

// ================================================================
//  CalibrateStraight.cpp
//  Live, on-the-floor calibration of the four wheel-speed globals so
//  the robot drives STRAIGHT under car.Advance().
//
//  You drive the robot forward, watch which way it veers, and trim
//  individual wheels with the IR remote until it tracks straight.
//  Then press PRINT and copy the four values into your real sketch.
//
//  No encoders needed — your eyes are the feedback, so it must run on
//  the floor (not on blocks).
// ================================================================

// ── Hardware ──────────────────────────────────────────────────
#define RECV_PIN  A3
mecanumCar car(3, 2);

// ── The four library globals we are calibrating ───────────────
//   Naming assumption (matches RunStaightLine.cpp):
//     Upper = FRONT wheels, Lower = REAR wheels.
extern uint8_t speed_Upper_L;  // front-left
extern uint8_t speed_Lower_L;  // rear-left
extern uint8_t speed_Upper_R;  // front-right
extern uint8_t speed_Lower_R;  // rear-right

// ── IR codes (same remote map as the rest of the project) ─────
#define CMD_1     0x16  // cycle which wheel the +/- buttons act on
#define CMD_2     0x19  // + raise selected wheel speed
#define CMD_3     0x0D  // - lower selected wheel speed
#define CMD_4     0x0C  // START driving forward
#define CMD_5     0x18  // STOP driving (to reposition)
#define CMD_6     0x5E  // PRINT current speeds (the result)
#define CMD_7     0x08  // RESET all four to BASE_SPEED
#define CMD_STAR  0x42  // emergency stop

// ── Tuning constants ──────────────────────────────────────────
#define BASE_SPEED  80   // starting speed for all four wheels
#define STEP         2   // change per +/- press (raise for coarser)
#define SPEED_MAX  200   // safety cap (PWM is 0-255)

// ── Which target the +/- buttons act on ───────────────────────
enum Target { TGT_ALL, TGT_FL, TGT_FR, TGT_RL, TGT_RR, TGT_COUNT };
Target selected = TGT_ALL;
bool   driving  = false;

const __FlashStringHelper *targetName(Target t) {
  switch (t) {
    case TGT_ALL: return F("ALL (base speed)");
    case TGT_FL:  return F("Front-Left");
    case TGT_FR:  return F("Front-Right");
    case TGT_RL:  return F("Rear-Left");
    case TGT_RR:  return F("Rear-Right");
    default:      return F("?");
  }
}

// Clamp to a safe PWM range (uint8_t math would otherwise wrap at 0/255).
uint8_t clampSpeed(int v) {
  if (v < 0)         return 0;
  if (v > SPEED_MAX) return SPEED_MAX;
  return (uint8_t)v;
}

// Apply a delta to whichever target is selected. For ALL, the delta is
// added to each wheel so any per-wheel trims you already set are preserved.
void adjust(int delta) {
  switch (selected) {
    case TGT_FL: speed_Upper_L = clampSpeed(speed_Upper_L + delta); break;
    case TGT_FR: speed_Upper_R = clampSpeed(speed_Upper_R + delta); break;
    case TGT_RL: speed_Lower_L = clampSpeed(speed_Lower_L + delta); break;
    case TGT_RR: speed_Lower_R = clampSpeed(speed_Lower_R + delta); break;
    case TGT_ALL:
      speed_Upper_L = clampSpeed(speed_Upper_L + delta);
      speed_Upper_R = clampSpeed(speed_Upper_R + delta);
      speed_Lower_L = clampSpeed(speed_Lower_L + delta);
      speed_Lower_R = clampSpeed(speed_Lower_R + delta);
      break;
    default: break;
  }
}

void resetSpeeds() {
  speed_Upper_L = BASE_SPEED;
  speed_Upper_R = BASE_SPEED;
  speed_Lower_L = BASE_SPEED;
  speed_Lower_R = BASE_SPEED;
}

void printSpeeds() {
  Serial.println(F("---- current wheel speeds ----"));
  Serial.print(F("  Front-Left  (speed_Upper_L) = ")); Serial.println(speed_Upper_L);
  Serial.print(F("  Front-Right (speed_Upper_R) = ")); Serial.println(speed_Upper_R);
  Serial.print(F("  Rear-Left   (speed_Lower_L) = ")); Serial.println(speed_Lower_L);
  Serial.print(F("  Rear-Right  (speed_Lower_R) = ")); Serial.println(speed_Lower_R);
  Serial.print(F("  paste -> speed_Upper_L="));  Serial.print(speed_Upper_L);
  Serial.print(F("; speed_Lower_L="));           Serial.print(speed_Lower_L);
  Serial.print(F("; speed_Upper_R="));           Serial.print(speed_Upper_R);
  Serial.print(F("; speed_Lower_R="));           Serial.print(speed_Lower_R);
  Serial.println(F(";"));
}

void printMenu() {
  Serial.println(F("\n=== STRAIGHT-LINE WHEEL CALIBRATION ==="));
  Serial.println(F("  1: select next wheel (ALL/FL/FR/RL/RR)"));
  Serial.println(F("  2: + speed of selected     3: - speed of selected"));
  Serial.println(F("  4: START forward           5: STOP"));
  Serial.println(F("  6: PRINT values            7: RESET all to base"));
  Serial.println(F("  *: emergency stop"));
  Serial.print  (F("  step=")); Serial.print(STEP);
  Serial.print  (F("  base=")); Serial.println(BASE_SPEED);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  car.Init();
  IrReceiver.begin(RECV_PIN, false);
  resetSpeeds();

  printMenu();
  Serial.print(F("Selected: ")); Serial.println(targetName(selected));
  printSpeeds();
  Serial.println(F("Place robot on the floor at a start line, press 4 to drive."));
}

void loop() {
  // --- handle one remote press per arrival ---
  if (IrReceiver.decode()) {
    uint8_t key = IrReceiver.decodedIRData.command;
    IrReceiver.resume();

    switch (key) {
      case CMD_1:
        selected = (Target)((selected + 1) % TGT_COUNT);
        Serial.print(F("Selected: ")); Serial.println(targetName(selected));
        break;
      case CMD_2: adjust(+STEP); printSpeeds(); break;
      case CMD_3: adjust(-STEP); printSpeeds(); break;
      case CMD_4: driving = true;  Serial.println(F(">> DRIVING forward")); break;
      case CMD_5: driving = false; car.Stop(); Serial.println(F(">> STOPPED")); break;
      case CMD_6: printSpeeds(); break;
      case CMD_7: resetSpeeds(); Serial.println(F(">> reset all to base")); printSpeeds(); break;
      case CMD_STAR: driving = false; car.Stop(); Serial.println(F(">> EMERGENCY STOP")); break;
      default: break;
    }
  }

  // --- re-assert drive state every pass (guards a dropped I2C frame) ---
  if (driving) {
    car.Advance();          // reads the four speed_* globals we are tuning
  } else {
    car.Stop();
  }
  delay(20);
}
