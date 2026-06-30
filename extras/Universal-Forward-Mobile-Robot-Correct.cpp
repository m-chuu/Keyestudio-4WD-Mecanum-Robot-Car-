#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.hpp>

// ── Hardware pins ─────────────────────────────────────────────
#define RECV_PIN        A3
#define SENSOR_LEFT     A0   // left line sensor
#define SENSOR_MID      A1   // middle sensor — should stay on the black line
#define SENSOR_RIGHT    A2   // right line sensor

// ── Cached sensor readings (updated once per loop) ───────────
static int sensorLeftVal = 0;
static int sensorMidVal = 0;
static int sensorRightVal = 0;

// ── EXTERN LIBRARY VARIABLES ──────────────────────────────────
extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

// ── IR Codes ──────────────────────────────────────────────────
#define CMD_3           0x0D // Press '3' = follow line, stop after N junctions
#define CMD_5           24   // Press '5' = start homing sequence
#define CMD_STAR        0x42 // '*' = emergency stop

// ── Drive configuration ───────────────────────────────────────
// 1 = all four wheels drive AND steer (rears mirror the front speeds).
// Set to 0 for front-wheel-only drive (e.g. if a rear wheel is loose).
#define DRIVE_REAR_WHEELS  1

// Forward base speeds, per side, from the straight-line calibration.
// The RIGHT wheels run stronger, so the right base is trimmed below the left
// (straight-line values L=80 / R=74). Used directly as the base speeds.
#define LINE_SPEED_L     80  //80// left  base (0-255)
#define LINE_SPEED_R     68  //74// right base (calibrated straight-line value)
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
// straddling a horizontal cross-line (a junction).
static bool allOnBlack() {
  return (sensorLeftVal > BLACK_THRESHOLD) && (sensorMidVal > BLACK_THRESHOLD) && (sensorRightVal > BLACK_THRESHOLD);
}

// Inline helpers using cached values
static inline bool onLeftBlack() {
  return sensorLeftVal > BLACK_THRESHOLD;
}

static inline bool onMidBlack() {
  return sensorMidVal > BLACK_THRESHOLD;
}

static inline bool onRightBlack() {
  return sensorRightVal > BLACK_THRESHOLD;
}

// Read all sensors once and cache values
static inline void readSensors() {
  sensorLeftVal = analogRead(SENSOR_LEFT);
  sensorMidVal = analogRead(SENSOR_MID);
  sensorRightVal = analogRead(SENSOR_RIGHT);
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
//  SYSTEM SELF-AUDIT DIAGNOSTIC
// ================================================================
bool RunSelfAudit() {
  Serial.println(F("\n============================================="));
  Serial.println(F("         SYSTEM SELF-AUDIT IN PROGRESS        "));
  Serial.println(F("============================================="));
  delay(500);

  // Audit: IR Line Tracking Sensors Input Check
  Serial.print(F("[AUDIT] Checking IR Array Pin States... "));
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MID, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  // Reading baseline to check if pins are floating or shorted
  readSensors();

  if (sensorLeftVal < 50 && sensorMidVal < 50 && sensorRightVal < 50) {
    Serial.println(F("WARNING: All IR pins pulling LOW. Check sensor power/connections."));
  } else if (sensorLeftVal > 900 && sensorMidVal > 900 && sensorRightVal > 900) {
    Serial.println(F("WARNING: All IR pins maxed out. Check for dust/contamination."));
  } else {
    Serial.println(F("OK"));
  }

  // Verify motor controller is responding on I2C
  Serial.print(F("[AUDIT] Checking Motor Controller on I2C... "));
  car.IIC_Start();
  car.IIC_SendByte(0x30 << 1);          // address + write bit
  bool noAck = car.IIC_RecvACK();        // 0 = ACK (device present), 1 = NAK (no device)
  car.IIC_Stop();

  if (noAck) {
    Serial.println(F("FAIL - Check SDA/SCL pins and controller power."));
  } else {
    Serial.println(F("OK"));
  }

  // Test drive command at low speed
  Serial.print(F("[AUDIT] Testing Drive Motors... "));
  speed_Upper_L = 50; speed_Lower_L = 50;
  speed_Upper_R = 50; speed_Lower_R = 50;
  driveFront(50, 50);
  delay(150);
  car.Stop();
  delay(100);
  Serial.println(F("OK"));

  Serial.println(F("============================================="));
  Serial.println(F(" AUDIT COMPLETE: System cleared for Homing.   "));
  Serial.println(F("=============================================\n"));
  return true;
}

// ================================================================
//  LINE FOLLOWING: Bang-bang steering with customizable base speeds
//
//  Uses outer sensors (onLeft, onRight) to apply steering via TURN_SLOW_SPEED
//  on the inner wheels while maintaining the provided base speeds.
//
//   • When centered or line lost: drive straight at baseSpeedL and baseSpeedR
//   • When drifted: slow the inner wheels to TURN_SLOW_SPEED
//   • Updates global speed variables before each motor command
// ================================================================
void FollowLine(bool onLeft, bool onRight, uint8_t baseSpeedL, uint8_t baseSpeedR) {
  if (onRight) {
    // drifted left -> steer RIGHT (slow the right wheels)
    speed_Upper_L = baseSpeedL;
    speed_Lower_L = baseSpeedL;
    speed_Upper_R = TURN_SLOW_SPEED;
    speed_Lower_R = TURN_SLOW_SPEED;
    driveFront(baseSpeedL, TURN_SLOW_SPEED);
  } else if (onLeft) {
    // drifted right -> steer LEFT (slow the left wheels)
    speed_Upper_L = TURN_SLOW_SPEED;
    speed_Lower_L = TURN_SLOW_SPEED;
    speed_Upper_R = baseSpeedR;
    speed_Lower_R = baseSpeedR;
    driveFront(TURN_SLOW_SPEED, baseSpeedR);
  } else {
    // centered OR line lost -> drive straight at base speeds
    speed_Upper_L = baseSpeedL;
    speed_Lower_L = baseSpeedL;
    speed_Upper_R = baseSpeedR;
    speed_Lower_R = baseSpeedR;
    driveFront(baseSpeedL, baseSpeedR);
  }
}

// ================================================================
//  HOMING SEQUENCE: Drive to black-line anchor
// ================================================================
void HomeRobot() {
  Serial.println(F("Entering Homing Mode..."));
  Serial.println(F("Searching for Black Line Anchor..."));

  bool lineFound = false;
  unsigned long timeoutGate = millis();
  unsigned long lastReport = 0;

  while (!lineFound) {
    // Read all sensors once at the start of the loop
    readSensors();

    // Actively steer toward the line using outer sensors
    bool onLeft  = onLeftBlack();
    bool onRight = onRightBlack();
    FollowLine(onLeft, onRight, LINE_SPEED_L, LINE_SPEED_R);

    // Check if all three sensors are on black line simultaneously
    if (allOnBlack()) {
      car.Stop();
      lineFound = true;
      Serial.println(F("-> Black Line Detected! Halting Drive Motors."));
    }

    // Diagnostic: print sensor states twice per second
    if (millis() - lastReport > 100) {
      lastReport = millis();
      Serial.print(F("[HOMING] IR L/C/R = "));
      Serial.print(onLeftBlack()  ? 1 : 0); Serial.print(F(" "));
      Serial.print(onMidBlack()   ? 1 : 0); Serial.print(F(" "));
      Serial.print(onRightBlack() ? 1 : 0);
      Serial.println(F("   (motors commanded via FollowLine)"));
    }

    // Safety fallback: timeout after 30 seconds
    if (millis() - timeoutGate > 30000) {
      car.Stop();
      Serial.println(F("-> Homing Error: Line seek timeout. System Halted."));
      while (true) { delay(1000); } // Lock down system for safety
    }

    delay(10);
  }

  // Clear IR remote command buffer to prevent accidental re-triggers
  Serial.print(F("Purging IR remote receiver buffers... "));
  while (IrReceiver.decode()) {
    IrReceiver.resume();
    delay(50);
  }
  Serial.println(F("Done."));

  // Handshake: System is ready and idle after homing
  Serial.println(F("{\"status\":\"READY\",\"state\":\"IDLE\"}"));
  restoreIR();
}

// ================================================================
//  FOLLOW LINE, STOP AFTER N JUNCTIONS  (bang-bang steering + counting)
//
//  Crawls forward using bang-bang steering (via FollowLine) while watching
//  for horizontal cross-lines. Each time all three sensors hit black at once
//  (allOnBlack) a new junction is counted; the car stops when target is reached.
//
//   • onJunction starts TRUE so the line under the car at launch isn't counted.
//   • It re-arms (onJunction = false) only after the car is back on a plain
//     vertical line (middle sensor only), so one wide cross-line counts once.
//   • Line lost (no sensor on black) -> FollowLine() keeps driving straight.
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
    // Read all sensors once at the start of the loop
    readSensors();

    bool onLeft  = onLeftBlack();
    bool onRight = onRightBlack();

    // --- Non-blocking debug: report sensor states every 500 ms ---
    if (millis() - lastReport > 500) {
      lastReport = millis();
      Serial.print(F("[SEARCH] IR L/C/R = "));
      Serial.print(onLeftBlack()  ? 1 : 0); Serial.print(F(" "));
      Serial.print(onMidBlack()   ? 1 : 0); Serial.print(F(" "));
      Serial.print(onRightBlack() ? 1 : 0);
      Serial.print(F("   (count="));
      Serial.print(count);
      Serial.println(F(", motors via FollowLine)"));
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
      FollowLine(onLeft, onRight, LINE_SPEED_L, LINE_SPEED_R);
    }
    // --- NOT a junction: steer to stay on the line ---
    else {
      // Re-arm the counter once back on a plain vertical line (middle only).
      if (!onLeft && !onRight) {
        onJunction = false;
      }

      FollowLine(onLeft, onRight, LINE_SPEED_L, LINE_SPEED_R);
    }

    if (emergencyStopPressed()) {
      Serial.println(F("  * Emergency stop pressed."));
      break;
    }
  }

  car.Stop();
  Serial.print(F("Counting STOP - junctions crossed: "));
  Serial.println(count);

  // Handshake: System is ready and idle after operation completes
  Serial.println(F("{\"status\":\"READY\",\"state\":\"IDLE\"}"));
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

  // Initialization sequence: audit -> wait for homing trigger
  RunSelfAudit();

  Serial.println(F("\nWaiting for operator input..."));
  Serial.println(F("'5' = start Homing sequence, '3' = follow line + stop after 3 junctions, '*' = emergency stop."));
}

void loop() {
  if (IrReceiver.decode()) {
    uint8_t key = IrReceiver.decodedIRData.command;
    IrReceiver.resume();

    if (key == CMD_5) {
      HomeRobot();  // drive to black-line anchor and enter ready state
    }
    else if (key == CMD_3) {
      Serial.println(F("{\"status\":\"BUSY\",\"state\":\"BUSY\"}"));
      stopByCounting(3);   // follow line, stop after 3 junctions
    }
  }
  delay(50);
}
