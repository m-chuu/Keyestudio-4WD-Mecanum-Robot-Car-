#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ===================== CHANNELS =====================
const int GRIPPER_CH  = 0;
const int WRIST_R_CH  = 1;
const int SHOULDER_CH = 2;
const int ELBOW_CH    = 3;
const int WRIST_P_CH  = 4;
const int BASE_CH     = 5;

// ===================== LIMITS =====================
struct JointLimits { int min; int max; };

const JointLimits LIMIT_G  = {0, 180};
const JointLimits LIMIT_B  = {10, 260};
const JointLimits LIMIT_S  = {20, 160};
const JointLimits LIMIT_E  = {20, 160};
const JointLimits LIMIT_WP = {10, 170};
const JointLimits LIMIT_WR = {10, 170};

// ===================== POSE =====================
struct Pose {
  int g, b, s, e, wp, wr;
};

struct NamedPose {
  const char* name;
  Pose pose;
};

// ===================== HARD-CODED SEQUENCE =====================
NamedPose sequence[] = {

  {"START", {
    30,   // GRIPPER
    135,  // BASE
    90,   // SHOULDER
    90,   // ELBOW
    90,   // WRIST_PITCH
    180   // WRIST_ROLL
  }},

  {"APPROACH", {
    30,
    140,
    70,
    110,
    85,
    90
  }},

  {"ALIGN", {
    30,
    145,
    55,
    135,
    80,
    90
  }},

  {"TAP", {
    30,
    145,
    45,
    150,
    75,
    90
  }},

  {"RETRACT", {
    30,
    140,
    70,
    120,
    85,
    90
  }},

  {"HOME", {
    30,
    135,
    90,
    90,
    90,
    90
  }}
};

const int seqCount = sizeof(sequence) / sizeof(sequence[0]);

// ===================== STATE =====================
int curG=30, curB=135, curS=90, curE=90, curWP=90, curWR=90;
bool isRunning = false;

// ===================== PWM =====================
void setServoSafe(uint8_t ch, int angle, JointLimits limit, int range) {
  int safeAngle = constrain(angle, limit.min, limit.max);
  pwm.writeMicroseconds(ch, map(safeAngle, 0, range, 500, 2500));
}

void updateRobot() {
  setServoSafe(GRIPPER_CH,  curG,  LIMIT_G,  180);
  setServoSafe(BASE_CH,     curB,  LIMIT_B,  270);
  setServoSafe(SHOULDER_CH, curS,  LIMIT_S,  180);
  setServoSafe(ELBOW_CH,    curE,  LIMIT_E,  180);
  setServoSafe(WRIST_P_CH,  curWP, LIMIT_WP, 180);
  setServoSafe(WRIST_R_CH,  curWR, LIMIT_WR, 180);
}

// ===================== MOTION ENGINE =====================
void moveToPose(Pose a, Pose b, int steps = 60, int delayMs = 25) {

  for (int i = 0; i <= steps; i++) {

    float t = (float)i / steps;

    curG  = a.g  + (b.g  - a.g)  * t;
    curB  = a.b  + (b.b  - a.b)  * t;
    curS  = a.s  + (b.s  - a.s)  * t;
    curE  = a.e  + (b.e  - a.e)  * t;
    curWP = a.wp + (b.wp - a.wp) * t;
    curWR = a.wr + (b.wr - a.wr) * t;

    updateRobot();
    delay(delayMs);
  }
}

// ===================== RUN SEQUENCE =====================
void runScript() {

  if (isRunning) return;
  isRunning = true;

  Serial.println("\n>>> RUNNING SEQUENCE <<<");

  for (int i = 0; i < seqCount - 1; i++) {

    Serial.print("Step: ");
    Serial.println(sequence[i].name);

    moveToPose(sequence[i].pose, sequence[i + 1].pose);
  }

  Serial.println(">>> DONE <<<");
  isRunning = false;
}

// ===================== SETUP =====================
void setup() {

  Serial.begin(115200);
  Wire.begin(21, 22);

  pwm.begin();
  pwm.setPWMFreq(50);

  updateRobot();

  Serial.println("SYSTEM READY");
  Serial.println("Press ENTER to run sequence");
}

// ===================== LOOP =====================
void loop() {

  if (Serial.available()) {

    char c = Serial.read();

    // ENTER trigger (Windows '\r', Linux '\n')
    if (c == '\n' || c == '\r') {
      runScript();
    }
  }
}