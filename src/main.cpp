#include <Arduino.h>
#define DECODE_NEC
#include <IRremote.hpp>
#include "MecanumCar_v2.h"

// --- Hardware Pins ---
const uint8_t MOTOR_SDA_PIN = 3;
const uint8_t MOTOR_SCL_PIN = 2;
const uint8_t IR_PIN = A3;
const uint8_t ULTRASONIC_TRIG_PIN = 12;
const uint8_t ULTRASONIC_ECHO_PIN = 13;

// --- Speeds & Settings ---
const uint8_t CRUISE_SPEED = 100;
const uint8_t TURN_SPEED = 160;   
const uint8_t REMOTE_SPEED = 245; 
const uint8_t OBSTACLE_STOP_CM = 24;
const unsigned long REMOTE_COMMAND_HOLD_MS = 350;

// --- IR Remote Mappings (Fixed Layout Configuration) ---
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

// Assigned custom triggers based on your hardware layout
const int IR_KEY_9    = 90; // Button '9' -> Moves Diagonal Back-Right
const int IR_KEY_0    = 82; // Button '0' -> Triggers Diagram Showcase
const int IR_KEY_HASH = 74; // Button '#' -> Triggers Autonomous Mode
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
RobotMode mode = MODE_STOPPED;
Motion activeMotion = MOTION_STOP;
unsigned long lastRemoteCommandMs = 0;

// --- Function Prototypes ---
void setDriveSpeed(uint8_t speed);
void setMotion(Motion motion);
void stopRobot();
void handleRemote();
void handleRemoteKey(int key);
void runAutonomous();
void executeDiagramShowcase();
uint16_t readDistanceCm();
void avoidObstacle();

void setup() {
  Serial.begin(9600);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  car.Init();
  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
  delay(400);

  stopRobot();
  Serial.println(F("System Online - Version 1 Autonomous Engine Active"));
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

// --- IR Remote Logic ---

void handleRemote() {
  if (!IrReceiver.decode()) return;

  int key = IrReceiver.decodedIRData.command;
  IrReceiver.resume();

  if (key == 0) return;
  handleRemoteKey(key);
}

void handleRemoteKey(int key) {
  lastRemoteCommandMs = millis();

  // Mode Switches
  if (key == IR_KEY_OK_STOP || key == IR_KEY_5) {
    stopRobot();
    return;
  } else if (key == IR_KEY_HASH) { // '#' key kicks off Autonomous Mode
    mode = MODE_AUTONOMOUS;
    car.left_led(true);
    car.right_led(true);
    return;
  } else if (key == IR_KEY_0) { // '0' key kicks off Diagram Sequence
    mode = MODE_SHOWCASE;
    executeDiagramShowcase();
    return;
  }

  // Manual Directional Control
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
    case IR_KEY_9: setMotion(MOTION_DIAG_BR); break; // 9 moves Diagonal Back-Right
    
    default: break;
  }
}

// --- Reverted Version 1 Autonomous Logic (Button #) ---

void runAutonomous() {
  uint16_t frontCm = readDistanceCm();

  // Simple, direct If-Else decision mapping
  if (frontCm > 0 && frontCm < OBSTACLE_STOP_CM) {
    avoidObstacle();
  } else {
    // Keeps moving straight forward when nothing is in front
    setDriveSpeed(CRUISE_SPEED);
    setMotion(MOTION_FORWARD); 
  }
}

// --- Diagram Sequence Mode (Button 0) ---

void executeDiagramShowcase() {
  Serial.println(F("Executing Diagram Paths a -> f"));
  
  // a) Forward Straight
  setDriveSpeed(CRUISE_SPEED);
  setMotion(MOTION_FORWARD);
  delay(1200);

  // b) Strafe Right
  setMotion(MOTION_STRAFE_RIGHT);
  delay(1200);

  // c) Diagonal Forward-Right
  setMotion(MOTION_DIAG_FR);
  delay(1200);

  // d) Curved Turn Right
  speed_Upper_L = CRUISE_SPEED;
  speed_Lower_L = CRUISE_SPEED;
  speed_Upper_R = CRUISE_SPEED / 3;
  speed_Lower_R = CRUISE_SPEED / 3;
  car.Advance();
  delay(1500);

  // e) Spin Clockwise
  setDriveSpeed(TURN_SPEED);
  setMotion(MOTION_TURN_RIGHT);
  delay(1200);

  // f) Swing Turn Right
  speed_Upper_L = CRUISE_SPEED;
  speed_Lower_L = CRUISE_SPEED;
  speed_Upper_R = 0;
  speed_Lower_R = 0;
  car.Advance();
  delay(1500);

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

void avoidObstacle() {
  setMotion(MOTION_STOP);
  delay(120);

  setDriveSpeed(TURN_SPEED);
  setMotion(MOTION_STRAFE_RIGHT); 
  delay(600); 

  setMotion(MOTION_STOP);
}