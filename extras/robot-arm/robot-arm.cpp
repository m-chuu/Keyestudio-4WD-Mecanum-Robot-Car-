#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ===================== STATES =====================
enum RobotState {
  STATE_INIT,
  STATE_IDLE,
  STATE_ESTOP
};

RobotState robotState = STATE_INIT;

// ===================== INIT FLAG =====================
bool initPromptShown = false;

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

struct Sequence {
  const char* name;
  NamedPose* poses;
  int count;
};

// ===================== HOME =====================
const Pose HOME_POSE = {30, 158, 105, 90, 150, 110};

// ===================== STATE VARIABLES =====================
int curG=30, curB=158, curS=100, curE=80, curWP=140, curWR=110;

// ===================== E-STOP =====================
volatile bool estopTriggered = false;

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
struct Motion {
  bool active = false;
  Pose a;
  Pose b;
  int step = 0;
  int steps = 160;
  unsigned long last = 0;
  int interval = 12;
};

Motion motion;

float easeSCurve(float t) {
  return t * t * t * (t * (t * 6 - 15) + 10);
}

void startMotion(Pose a, Pose b, int steps = 160, int interval = 12) {
  motion.active = true;
  motion.a = a;
  motion.b = b;
  motion.steps = steps;
  motion.step = 0;
  motion.interval = interval;
  motion.last = millis();
}

void updateMotion() {
  if (!motion.active) return;
  if (robotState == STATE_ESTOP || estopTriggered) {
    motion.active = false;
    return;
  }
  if (millis() - motion.last < motion.interval) return;

  motion.last = millis();

  float t = (float)motion.step / motion.steps;
  t = easeSCurve(t);

  curG  = motion.a.g  + (motion.b.g  - motion.a.g)  * t;
  curB  = motion.a.b  + (motion.b.b  - motion.a.b)  * t;
  curS  = motion.a.s  + (motion.b.s  - motion.a.s)  * t;
  curE  = motion.a.e  + (motion.b.e  - motion.a.e)  * t;
  curWP = motion.a.wp + (motion.b.wp - motion.a.wp) * t;
  curWR = motion.a.wr + (motion.b.wr - motion.a.wr) * t;

  updateRobot();
  motion.step++;

  if (motion.step >= motion.steps) motion.active = false;
}

// ===================== EMERGENCY STOP =====================
void emergencyStop() {
  estopTriggered = true;
  robotState = STATE_ESTOP;
  motion.active = false;
  Serial.println("\n** EMERGENCY STOP **");
}

// ===================== HOME =====================
void goHome() {
  if (robotState == STATE_ESTOP) return;
  Serial.println("\n>>> GO HOME <<<");
  Pose start = {curG, curB, curS, curE, curWP, curWR};
  startMotion(start, HOME_POSE, 120, 10);
  while (motion.active) { updateMotion(); }
  Serial.println(">>> HOME REACHED <<<");
}

// ===================== SEQUENCES =====================
NamedPose bluePoses[] = {
  {"POSE_1", {30, 158, 105, 90, 150, 110}},
  {"POSE_2", {30, 158, 105, 90, 75, 110}},
  {"POSE_3", {30, 158, 40, 90, 75, 110}},
  {"POSE_4", {130, 158, 40, 90, 75, 110}},
  {"POSE_5", {130, 158, 85, 90, 75, 110}},
  {"POSE_6", {130, 123, 85, 90, 75, 110}},
  {"POSE_7", {130, 123, 65, 90, 75, 110}},
  {"POSE_8", {-5, 123, 65, 90, 75, 110}}
};
Sequence BLUE_SEQUENCE = {"BLUE", bluePoses, 8};

NamedPose redPoses[] = {
  {"POSE_1", {30, 158, 105, 90, 150, 110}},
  {"POSE_2", {30, 158, 105, 90, 90, 110}},
  {"POSE_3", {30, 158, 105, 155, 90, 110}},
  {"POSE_4", {125, 158, 105, 155, 90, 110}},
  {"POSE_5", {125, 158, 105, 155, 125, 110}},
  {"POSE_6", {125, 113, 105, 155, 125, 110}},
  {"POSE_7", {125, 113, 90, 155, 115, 110}},
  {"POSE_8", {0, 113, 90, 155, 115, 110}}
};
Sequence RED_SEQUENCE = {"RED", redPoses, 8};

NamedPose yellowPoses[] = {
  {"POSE_1", {30, 158, 105, 90, 150, 110}},
  {"POSE_2", {30, 158, 105, 90, 90, 110}},
  {"POSE_3", {30, 158, 105, 130, 90, 110}},
  {"POSE_4", {30, 158, 70, 130, 90, 110}},
  {"POSE_5", {115, 158, 70, 130, 90, 110}},
  {"POSE_6", {115, 158, 70, 130, 150, 110}},
  {"POSE_7", {115, 103, 70, 130, 150, 110}},
  {"POSE_8", {115, 98, 50, 165, 150, 110}},
  {"POSE_9", {-5, 98, 50, 165, 150, 110}}
};
Sequence YELLOW_SEQUENCE = {"YELLOW", yellowPoses, 9};

// ===================== RUN SEQUENCE =====================
void runSequence(Sequence &seq) {
  if (estopTriggered || robotState == STATE_ESTOP) return;
  Serial.printf("\n>>> RUN %s <<<\n", seq.name);
  for (int i = 0; i < seq.count - 1; i++) {
    startMotion(seq.poses[i].pose, seq.poses[i+1].pose);
    while (motion.active) {
      updateMotion();
      if (estopTriggered || robotState == STATE_ESTOP) return;
    }
  }
  Serial.println(">>> SEQUENCE COMPLETE <<<");
  goHome();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);
  updateRobot();
}

void loop() {
  updateMotion();
  switch (robotState) {
    case STATE_INIT:
      if (!initPromptShown) {
        Serial.println("\n======================");
        Serial.println(" ROBOT READY CHECK");
        Serial.println("======================");
        Serial.println("Is robot ready? (y/n)");
        initPromptShown = true;
      }
      if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        if (input == "y" || input == "yes") {
          Serial.println("Robot READY confirmed.");
          goHome();
          robotState = STATE_IDLE;
          Serial.println("IDLE READY");
          Serial.println("R=RED B=BLUE Y=YELLOW E=ESTOP H=HOME X=RESET");
        }
      }
      break;
    case STATE_IDLE:
      if (Serial.available()) {
        char c = tolower(Serial.read());
        if (c == 'r') runSequence(RED_SEQUENCE);
        if (c == 'b') runSequence(BLUE_SEQUENCE);
        if (c == 'y') runSequence(YELLOW_SEQUENCE);
        if (c == 'h') goHome();
        if (c == 'e') emergencyStop();
      }
      break;
    case STATE_ESTOP:
      updateRobot();
      if (Serial.available()) {
        char c = tolower(Serial.read());
        if (c == 'x') {
          Serial.println("RESET SYSTEM");
          estopTriggered = false;
          initPromptShown = false;
          robotState = STATE_INIT;
        }
      }
      break;
  }
}