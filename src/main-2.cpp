#include <Arduino.h>
#define DECODE_NEC
#include <IRremote.hpp>
#include <Servo.h>          
#include "MecanumCar_v2.h"

// --- Hardware Pins ---
const uint8_t MOTOR_SDA_PIN = 3;
const uint8_t MOTOR_SCL_PIN = 2;
const uint8_t IR_PIN = A3;
const uint8_t ULTRASONIC_TRIG_PIN = 12;
const uint8_t ULTRASONIC_ECHO_PIN = 13;
const uint8_t SERVO_PIN = A0;      

// --- Speeds & Settings ---
const uint8_t CRUISE_SPEED = 220;  // INCREASED SPEED: Boosted from 170 for faster travel
const uint8_t TURN_SPEED = 160;   
const uint8_t REMOTE_SPEED = 245; 
const uint8_t OBSTACLE_STOP_CM = 24;
const unsigned long REMOTE_COMMAND_HOLD_MS = 350;

// --- STRICT 2-POSITION SERVO CALIBRATION ---
const int SERVO_OPEN_ANGLE = 15;   // POSITION 1: Fully Open
const int SERVO_CLOSED_ANGLE = 85; // POSITION 2: Fully Closed (Grip)

// --- IR Remote Mappings ---
const int IR_KEY_UP      = 70;
const int IR_KEY_DOWN    = 21;
const int IR_KEY_LEFT    = 68;
const int IR_KEY_RIGHT   = 67;
const int IR_KEY_OK_STOP = 64; 

const int IR_KEY_1 = 22; 
const int IR_KEY_2 = 25; 
const int IR_KEY_3 = 13; 
const int IR_KEY_4 = 12; 
const int IR_KEY_5 = 24; 
const int IR_KEY_6 = 94; 
const int IR_KEY_7 = 8;  
const int IR_KEY_8 = 28; 

const int IR_KEY_9    = 90; 
const int IR_KEY_0    = 82; 
const int IR_KEY_HASH = 74; 
const int IR_KEY_STAR = 66; 

// --- Enums ---
enum RobotMode {
  MODE_STOPPED,
  MODE_MANUAL,
  MODE_AUTONOMOUS,
  MODE_SHOWCASE
};

enum Motion {
  MOTION_STOP,
  MOTION_FORWARD,
  MOTION_BACK,
  MOTION_TURN_LEFT,
  MOTION_TURN_RIGHT,
  MOTION_STRAFE_LEFT,
  MOTION_STRAFE_RIGHT,
  MOTION_DIAG_FL, 
  MOTION_DIAG_FR, 
  MOTION_DIAG_BL, 
  MOTION_DIAG_BR  
};

// --- Globals ---
mecanumCar car(MOTOR_SDA_PIN, MOTOR_SCL_PIN);
Servo clamperServo;                
RobotMode mode = MODE_STOPPED;
Motion activeMotion = MOTION_STOP;
unsigned long lastRemoteCommandMs = 0;
bool isClamped = false;            

// --- Function Prototypes ---
void setDriveSpeed(uint8_t speed);
void setMotion(Motion motion);
void stopRobot();
void handleRemote();
void handleRemoteKey(int key);
void toggleClamper();              
void runAutonomous();
void executeDiagramShowcase();
bool delayWithCheck(unsigned long ms);
uint16_t readDistanceCm();

void setup() {
  Serial.begin(9600);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  clamperServo.attach(SERVO_PIN);
  clamperServo.write(SERVO_OPEN_ANGLE); 

  car.Init();
  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
  delay(400);

  stopRobot();
  Serial.println(F("System Online - Fast Autonomous Stop Mode Active"));
}

void loop() {
  handleRemote();

  if (mode == MODE_AUTONOMOUS) {
    runAutonomous();
  } else if (mode == MODE_MANUAL && activeMotion != MOTION_STOP &&
             millis() - lastRemoteCommandMs > REMOTE_COMMAND_HOLD_MS) {
    stopRobot();
  }
}

// --- Movement Controllers ---

void setDriveSpeed(uint8_t speed) {
  speed_Upper_L = speed;
  speed_Lower_L = speed;
  speed_Upper_R = speed;
  speed_Lower_R = speed;
}

void setMotion(Motion motion) {
  activeMotion = motion;

  switch (motion) {
    case MOTION_FORWARD:      car.Advance(); break;
    case MOTION_BACK:         car.Back(); break;
    case MOTION_TURN_LEFT:    car.Turn_Left(); break;
    case MOTION_TURN_RIGHT:   car.Turn_Right(); break;
    case MOTION_STRAFE_LEFT:  car.L_Move(); break;
    case MOTION_STRAFE_RIGHT: car.R_Move(); break;
    case MOTION_DIAG_FL:      car.LU_Move(); break;
    case MOTION_DIAG_FR:      car.RU_Move(); break;
    case MOTION_DIAG_BL:      car.LD_Move(); break; 
    case MOTION_DIAG_BR:      car.RD_Move(); break; 
    case MOTION_STOP:
    default:                  car.Stop(); break;
  }
}

void stopRobot() {
  setMotion(MOTION_STOP);
  mode = MODE_STOPPED;
  car.left_led(false);
  car.right_led(false);
}

// --- Strict 2-Position Toggle Logic ---
void toggleClamper() {
  isClamped = !isClamped;
  if (isClamped) {
    clamperServo.write(SERVO_CLOSED_ANGLE);
    Serial.println(F("Clamper State: CLOSED"));
  } else {
    clamperServo.write(SERVO_OPEN_ANGLE);
    Serial.println(F("Clamper State: OPEN"));
  }
}

// --- IR Remote Logic ---

void handleRemote() {
  if (!IrReceiver.decode()) return;

  int key = IrReceiver.decodedIRData.command;
  IrReceiver.resume();

  if (key == 0) return;
  handleRemoteKey(key);
}

void handleRemoteKey(int key) {
  if (key == IR_KEY_STAR) {
    toggleClamper();
    return;
  }

  lastRemoteCommandMs = millis();

  if (key == IR_KEY_OK_STOP || key == IR_KEY_5) {
    stopRobot();
    return;
  } else if (key == IR_KEY_HASH) { 
    mode = MODE_AUTONOMOUS;
    car.left_led(true);
    car.right_led(true);
    return;
  } else if (key == IR_KEY_0) { 
    mode = MODE_SHOWCASE;
    executeDiagramShowcase();
    return;
  }

  mode = MODE_MANUAL;
  setDriveSpeed(REMOTE_SPEED);
  car.left_led(false);
  car.right_led(true);

  switch (key) {
    case IR_KEY_UP:    setMotion(MOTION_FORWARD); break;
    case IR_KEY_DOWN:  setMotion(MOTION_BACK); break;
    case IR_KEY_LEFT:  setMotion(MOTION_TURN_LEFT); break;
    case IR_KEY_RIGHT: setMotion(MOTION_TURN_RIGHT); break;

    case IR_KEY_1: setMotion(MOTION_DIAG_FL); break;
    case IR_KEY_2: setMotion(MOTION_FORWARD); break;
    case IR_KEY_3: setMotion(MOTION_DIAG_FR); break;
    case IR_KEY_4: setMotion(MOTION_STRAFE_LEFT); break;
    case IR_KEY_6: setMotion(MOTION_STRAFE_RIGHT); break;
    case IR_KEY_7: setMotion(MOTION_DIAG_BL); break;
    case IR_KEY_8: setMotion(MOTION_BACK); break;
    case IR_KEY_9: setMotion(MOTION_DIAG_BR); break; 
    
    default: break;
  }
}

// --- Version 1 Autonomous Logic (Button #) ---

void runAutonomous() {
  uint16_t frontCm = readDistanceCm();

  // If an object is detected inside the threshold, halt instantly
  if (frontCm > 0 && frontCm < OBSTACLE_STOP_CM) {
    setMotion(MOTION_STOP); 
  } else {
    // Otherwise, cruise forward at the new, higher speed setting
    setDriveSpeed(CRUISE_SPEED);
    setMotion(MOTION_FORWARD); 
  }
}

// --- Custom Non-Blocking Delay & Kill Switch Monitor ---
bool delayWithCheck(unsigned long ms) {
  unsigned long startMs = millis();
  
  while (millis() - startMs < ms) {
    if (IrReceiver.decode()) {
      int key = IrReceiver.decodedIRData.command;
      IrReceiver.resume();
      
      if (key == IR_KEY_STAR) {
        toggleClamper();
      }
      else if (key == IR_KEY_OK_STOP || key == IR_KEY_5) {
        stopRobot();
        Serial.println(F("<< SHOWCASE KILLED BY USER >>"));
        return true; 
      }
    }
    delay(10); 
  }
  return false; 
}

// --- Diagram Sequence Mode (Button 0) ---

void executeDiagramShowcase() {
  Serial.println(F("Executing Diagram Paths a -> f"));
  
  // a) Forward Straight
  setDriveSpeed(CRUISE_SPEED);
  setMotion(MOTION_FORWARD);
  if (delayWithCheck(1200)) return;

  // b) Strafe Right
  setMotion(MOTION_STRAFE_RIGHT);
  if (delayWithCheck(1200)) return;

  // c) Diagonal Forward-Right
  setMotion(MOTION_DIAG_FR);
  if (delayWithCheck(1200)) return;

  // d) Curved Turn Right
  speed_Upper_L = CRUISE_SPEED;
  speed_Lower_L = CRUISE_SPEED;
  speed_Upper_R = CRUISE_SPEED / 3;
  speed_Lower_R = CRUISE_SPEED / 3;
  car.Advance();
  if (delayWithCheck(1500)) return;

  // e) Spin Clockwise
  setDriveSpeed(TURN_SPEED);
  setMotion(MOTION_TURN_RIGHT);
  if (delayWithCheck(1200)) return;

  // f) Swing Turn Right
  speed_Upper_L = CRUISE_SPEED;
  speed_Lower_L = CRUISE_SPEED;
  speed_Upper_R = 0;
  speed_Lower_R = 0;
  car.Advance();
  if (delayWithCheck(1500)) return;

  stopRobot();
}

// --- Sensors & Utilities ---

uint16_t readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long echoUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000UL);
  if (echoUs == 0) return 0;

  return static_cast<uint16_t>(echoUs / 58UL);
}