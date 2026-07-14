#include <Arduino.h>
#include <IRremote.hpp>
#include <Servo.h>

// ================= IR REMOTE SETTING =================
#define IR_RECEIVE_PIN A3

// Button codes for KS0560 remote
#define BUTTON_1_CODE           0x16  // Toggle clamp open/close
#define BUTTON_2_CODE           0x19  // "2" -> speed UP  (TODO: confirm via Serial Monitor)
#define BUTTON_3_CODE           0xD  // "3" -> speed DOWN (TODO: confirm via Serial Monitor)

// ================= SERVO SETTING =================
#define SERVO_PIN 9

Servo clampServo;

const int CLAMP_OPEN_ANGLE  = 30;
const int CLAMP_CLOSE_ANGLE = 90;

const int SERVO_STEP_SIZE  = 1;

// Speed is adjustable at runtime via IR remote.
// Lower delay = faster movement, higher delay = slower movement.
const int SERVO_DELAY_MIN  = 0;   // fastest allowed (ms per step)
const int SERVO_DELAY_MAX  = 40;  // slowest allowed (ms per step)
const int SERVO_DELAY_STEP = 2;   // how much each button press changes delay

// Time to let the servo settle into its final position before cutting power (limp mode)
const int LIMP_SETTLE_DELAY = 200; // ms

int currentServoDelay = 0; // starting speed

int clampCurrentAngle = CLAMP_CLOSE_ANGLE;
bool clampIsOpen = false;
bool servoAttached = false;

// ================= CLAMP FUNCTIONS =================
void moveClampSmooth(int targetAngle) {
  // Make sure the servo has power before moving (it may be limp/detached)
  if (!servoAttached) {
    clampServo.attach(SERVO_PIN);
    servoAttached = true;
  }

  int step = (targetAngle >= clampCurrentAngle) ? SERVO_STEP_SIZE : -SERVO_STEP_SIZE;

  while (clampCurrentAngle != targetAngle) {
    clampCurrentAngle += step;
    if ((step > 0 && clampCurrentAngle > targetAngle) ||
        (step < 0 && clampCurrentAngle < targetAngle)) {
      clampCurrentAngle = targetAngle;
    }
    clampServo.write(clampCurrentAngle);
    delay(currentServoDelay);
  }
  // NOTE: no auto-detach here. Caller (openClamp/closeClamp) decides,
  // since closing means gripping + carrying a load and must hold torque.
}

void openClamp() {
  moveClampSmooth(CLAMP_OPEN_ANGLE);
  clampIsOpen = true;
  Serial.println("Clamp OPEN");

  // Safe to go limp here — nothing is being gripped, no load to hold
  delay(LIMP_SETTLE_DELAY);
  clampServo.detach();
  servoAttached = false;
  Serial.println("Servo detached (limp mode) - idle/open state");
}

void closeClamp() {
  moveClampSmooth(CLAMP_CLOSE_ANGLE);
  clampIsOpen = false;
  Serial.println("Clamp CLOSE - gripping object, holding torque maintained");
  // Intentionally NOT detaching here — object is being carried,
  // servo must stay attached to hold grip position under load.
}

void toggleClamp() {
  if (clampIsOpen) {
    closeClamp();
  } else {
    openClamp();
  }
}

// ================= SPEED CONTROL =================
void increaseSpeed() {
  // Increasing speed means DECREASING the delay
  currentServoDelay -= SERVO_DELAY_STEP;
  if (currentServoDelay < SERVO_DELAY_MIN) {
    currentServoDelay = SERVO_DELAY_MIN;
  }
  Serial.print("Speed UP (Button 2) -> delay: ");
  Serial.println(currentServoDelay);
}

void decreaseSpeed() {
  // Decreasing speed means INCREASING the delay
  currentServoDelay += SERVO_DELAY_STEP;
  if (currentServoDelay > SERVO_DELAY_MAX) {
    currentServoDelay = SERVO_DELAY_MAX;
  }
  Serial.print("Speed DOWN (Button 3) -> delay: ");
  Serial.println(currentServoDelay);
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  clampServo.attach(SERVO_PIN);
  servoAttached = true;
  closeClamp(); // starts in gripped/closed state, stays attached

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  Serial.println("KS0560 Servo Clamp Ready");
  Serial.println("Press Button 1 to Open/Close Clamp");
  Serial.println("Press Button 2/3 to adjust Servo Speed");
}

// ================= LOOP =================
void loop() {
  if (IrReceiver.decode()) {

    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {

      uint8_t command = IrReceiver.decodedIRData.command;

      Serial.print("IR Command: 0x");
      Serial.println(command, HEX);

      if (command == BUTTON_1_CODE) {
        toggleClamp();
      } else if (command == BUTTON_2_CODE) {
        increaseSpeed();
      } else if (command == BUTTON_3_CODE) {
        decreaseSpeed();
      }
    }

    IrReceiver.resume();
  }
}