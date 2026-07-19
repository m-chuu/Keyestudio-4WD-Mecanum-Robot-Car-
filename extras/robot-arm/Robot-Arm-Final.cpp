#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>
#include "HUSKYLENS.h"
#include "../../secrets.h"  // WiFi & Favoriot credentials (gitignored)

// =====================================================================
// FAVORIOT CLOUD TRIGGER SETUP GUIDE (one-time, done once per account):
//
// 1. Sign up / log in at https://platform.favoriot.com
//
// 2. Go to the "Devices" page and create TWO devices:
//      - one for the mobile robot, e.g. device_developer_id "mobilebot@yourusername"
//      - one for this arm, e.g. device_developer_id "armbot@yourusername"
//    (device_developer_id is just a name you choose + "@" + your account name)
//
// 3. Go to Account Settings (top-right dropdown) and copy your API Key
//    (a long alphanumeric string). Paste it into FAVORIOT_API_KEY below.
//
// 4. On the MOBILE ROBOT's own code, after it finishes dropping the object
//    at the dropping area, have it POST a stream entry like this:
//
//      POST https://apiv2.favoriot.com/v2/streams
//      Headers: apikey: <API KEY>, Content-Type: application/json
//      Body: {"device_developer_id":"mobilebot@yourusername","data":{"distance":"TaskMiddle_complete"}}
//
//    The arm looks at the "distance" field of the latest stream entry and
//    treats ANY value ending in "_complete" (e.g. TaskMiddle_complete,
//    TaskLeft_complete, TaskRight_complete) as "go scan and sort".
//
// 5. This arm sketch periodically GETs the mobile robot's latest stream
//    entry, and if it's a new one ending in "_complete" it hasn't reacted
//    to yet, calls scanAndSort().
// =====================================================================

// =====================================================================
// HUSKYLENS SETUP GUIDE - TAG RECOGNITION (do this once, on the HuskyLens
// screen, before running this sketch):
//
// 1. Power the HuskyLens. Go to General Settings (swipe/menu) and set
//    "Protocol Type" to UART (it is wired to ESP32 RX2/TX2 here).
//    Also check the UART baud rate shown there — it must match
//    HUSKY_BAUD below (default HuskyLens baud is 9600).
//
// 2. On the main screen, swipe to select the "Tag Recognition" function/
//    algorithm.
//
// 3. Unlike Color Recognition, tags are NOT taught with the learning
//    button - each physical AprilTag (family tag36h11, which HuskyLens
//    ships/recognizes by default) already has a fixed ID printed into its
//    pattern. Print or obtain 3 tags with different IDs and attach one to
//    each object:
//      - Tag with ID 1 -> object for the RED bin
//      - Tag with ID 2 -> object for the BLUE bin
//      - Tag with ID 3 -> object for the YELLOW bin
//    (Any 3 distinct tag IDs work - just make sure the COLOR_ID_* defines
//    below match whichever tag you physically attached to which object.)
//
// 4. Point the camera at a tagged object and HuskyLens will report
//    result.ID = that tag's number directly - no training step needed.
// =====================================================================

#define HUSKY_BAUD 9600
#define HUSKY_RX_PIN 16   // ESP32 RX2 <- HuskyLens TX
#define HUSKY_TX_PIN 17   // ESP32 TX2 -> HuskyLens RX

// These must match whichever AprilTag ID is physically attached to each
// colored object (see Tag Recognition setup guide above).
#define COLOR_ID_RED    1
#define COLOR_ID_BLUE   2
#define COLOR_ID_YELLOW 3

HUSKYLENS huskylens;
HardwareSerial HuskySerial(2); // ESP32 Serial2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ===================== WIFI / FAVORIOT CONFIG =====================
// WIFI_SSID2/WIFI_PASSWORD2 (training-room network), FAVORIOT_API_KEY and
// MOBILE_ROBOT_DEVICE_ID (must match whatever device_developer_id the
// MOBILE ROBOT posts to) all come from secrets.h at the project root.

const unsigned long FAVORIOT_POLL_INTERVAL_MS = 3000; // how often to check
unsigned long lastFavoriotPollTime = 0;
String lastSeenStreamId = ""; // tracks which stream entry we've already reacted to

WiFiClientSecure secureClient; // used for HTTPS calls to Favoriot

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
  {"POSE_2", {30, 158, 80, 165, 110, 110}},
  {"POSE_3", {115, 158, 80, 165, 110, 110}},
  {"POSE_4", {115, 158, 80, 165, 165, 110}},
  {"POSE_5", {115, 118, 80, 100, 85, 110}},
  {"POSE_6", {115, 123, 70, 100, 85, 110}},
  {"POSE_7", {5, 123, 70, 100, 85, 110}}
};
Sequence BLUE_SEQUENCE = {"BLUE", bluePoses, 7};

NamedPose redPoses[] = {
  {"POSE_1", {30, 158, 105, 90, 150, 110}},
  {"POSE_2", {30, 158, 75, 165, 110, 110}},
  {"POSE_3", {90, 158, 75, 165, 110, 110}},
  {"POSE_4", {90, 158, 75, 165, 150, 110}},
  {"POSE_5", {90, 113, 75, 165, 150, 110}},
  {"POSE_6", {90, 113, 75, 165, 140, 110}},
  {"POSE_7", {90, 113, 70, 165, 140, 110}},
  {"POSE_8", {90, 113, 70, 165, 135, 110}},
  {"POSE_9", {60, 113, 70, 165, 135, 110}}
};
Sequence RED_SEQUENCE = {"RED", redPoses, 9};

NamedPose yellowPoses[] = {
  {"POSE_1", {30, 158, 105, 90, 150, 110}},
  {"POSE_2", {30, 158, 75, 165, 110, 110}},
  {"POSE_3", {80, 158, 75, 165, 110, 110}},
  {"POSE_4", {80, 158, 75, 165, 180, 110}},
  {"POSE_5", {80, 98, 75, 165, 180, 110}},
  {"POSE_6", {80, 98, 55, 165, 150, 110}},
  {"POSE_7", {80, 98, 55, 165, 145, 110}},
  {"POSE_8", {50, 98, 55, 165, 145, 110}}
};
Sequence YELLOW_SEQUENCE = {"YELLOW", yellowPoses, 8};

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

// ===================== HUSKYLENS COLOR SCAN =====================
// Called on demand (serial command 's'). Requests one reading from the
// HuskyLens, matches the ID against the trained color mapping above,
// and runs the corresponding pick-and-place sequence.
void scanAndSort() {
  if (estopTriggered || robotState == STATE_ESTOP) return;

  Serial.println("\n>>> SCANNING FOR COLOR <<<");

  if (!huskylens.request()) {
    Serial.println("HUSKYLENS: request failed - check wiring/baud.");
    return;
  }
  if (!huskylens.available()) {
    Serial.println("HUSKYLENS: no tag detected in frame.");
    return;
  }

  HUSKYLENSResult result = huskylens.read();

  if (result.command != COMMAND_RETURN_BLOCK) {
    Serial.println("HUSKYLENS: unexpected result type, expected tag block.");
    return;
  }

  Serial.printf("HUSKYLENS: detected tag ID %d\n", result.ID);

  switch (result.ID) {
    case COLOR_ID_RED:
      runSequence(RED_SEQUENCE);
      break;
    case COLOR_ID_BLUE:
      runSequence(BLUE_SEQUENCE);
      break;
    case COLOR_ID_YELLOW:
      runSequence(YELLOW_SEQUENCE);
      break;
    default:
      Serial.printf("HUSKYLENS: ID %d has no matching bin - update COLOR_ID_* defines.\n", result.ID);
      break;
  }
}

// ===================== WIFI CONNECT =====================
void connectWiFi() {
  Serial.print("WIFI: connecting to ");
  Serial.println(WIFI_SSID2);
  WiFi.begin(WIFI_SSID2, WIFI_PASSWORD2);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WIFI: connected, IP = ");
  Serial.println(WiFi.localIP());
}

// ===================== FAVORIOT POLLING =====================
// Checks whether the mobile robot has posted a NEW stream entry since we
// last looked. If so, treats it as "go" and runs scanAndSort() once.
// This is non-blocking-ish: it only actually does the HTTP call once every
// FAVORIOT_POLL_INTERVAL_MS, so it's safe to call every loop() iteration.
void checkMobileRobotSignal() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastFavoriotPollTime < FAVORIOT_POLL_INTERVAL_MS) return;
  lastFavoriotPollTime = millis();

  secureClient.setInsecure(); // skip cert validation - fine for this project

  HTTPClient http;
  String url = String("https://apiv2.favoriot.com/v2/devices/")
               + MOBILE_ROBOT_DEVICE_ID
               + "/streams?max=1&order=desc";

  http.begin(secureClient, url); // pass the secure client explicitly
  http.addHeader("apikey", FAVORIOT_API_KEY);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.GET();

  if (httpCode != 200) {
    Serial.printf("FAVORIOT: poll failed, HTTP code %d\n", httpCode);
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc; // ArduinoJson v7 style; if using v6, use DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("FAVORIOT: JSON parse failed - ");
    Serial.println(err.c_str());
    return;
  }

  JsonArray results = doc["results"].as<JsonArray>();
  if (results.isNull() || results.size() == 0) {
    return; // no streams yet from the mobile robot
  }

  JsonObject latest = results[0];
  String streamId = latest["stream_id"] | "";
  String taskStatus = latest["data"]["distance"] | "";

  if (streamId.length() == 0 || streamId == lastSeenStreamId) {
    return; // nothing new
  }

  // Mark as seen regardless of content, so we never re-react to the same
  // stream entry twice even if it isn't a completion signal.
  lastSeenStreamId = streamId;

  if (taskStatus.endsWith("_complete")) {
    Serial.printf("FAVORIOT: task completed (%s) - triggering scan.\n", taskStatus.c_str());
    scanAndSort();
  } else {
    Serial.printf("FAVORIOT: new entry (%s) but not a completion signal - ignoring.\n", taskStatus.c_str());
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);
  updateRobot();

  HuskySerial.begin(HUSKY_BAUD, SERIAL_8N1, HUSKY_RX_PIN, HUSKY_TX_PIN);
  while (!huskylens.begin(HuskySerial)) {
    Serial.println("HUSKYLENS: begin failed - check wiring (RX2/TX2) and that");
    Serial.println("           Protocol Type on the HuskyLens is set to UART.");
    delay(500);
  }
  Serial.println("HUSKYLENS: connected.");
  huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);

  connectWiFi();
}

// ===================== LOOP =====================
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
          Serial.println("R=RED B=BLUE Y=YELLOW  S=SCAN&SORT  H=HOME  E=ESTOP  X=RESET");
        }
      }
      break;

    case STATE_IDLE:
      checkMobileRobotSignal(); // cloud trigger from the mobile robot
      if (Serial.available()) {
        char c = tolower(Serial.read());
        if (c == 'r') runSequence(RED_SEQUENCE);
        if (c == 'b') runSequence(BLUE_SEQUENCE);
        if (c == 'y') runSequence(YELLOW_SEQUENCE);
        if (c == 's') scanAndSort();   // manual trigger, same as cloud trigger
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