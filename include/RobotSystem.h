#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H

#include <Arduino.h>
#include <Servo.h>
#define DECODE_NEC         // Added to match your main configuration
#include <IRremote.hpp>    // FIX: Tells the compiler what IrReceiver is
#include "MecanumCar_v2.h"

// --- Verified hardware wiring (from working reference sketch) ---
// Motor controller bit-bang I2C lives on digital pins D3 (SDA) and D2 (SCL).
const uint8_t MOTOR_SDA_PIN = 3;
const uint8_t MOTOR_SCL_PIN = 2;
const uint8_t SERVO_PIN     = A4;   // Clamper servo signal (moved off A0 -> A0 is RIGHT line sensor)
const uint8_t IR_REMOTE_PIN = A3;   // IR remote receiver

// Line-tracking sensors -- VERIFIED via pin-finder scan (white~25, black~750).
// Servo was moved A0 -> A4 so RIGHT (A0) no longer conflicts. A5 still free.
const uint8_t IR_LINE_LEFT_PIN   = A2;
const uint8_t IR_LINE_CENTER_PIN = A1;
const uint8_t IR_LINE_RIGHT_PIN  = A0;

// Declare global hardware references used from main sketch
extern mecanumCar car;
extern Servo clamperServo;
const int SERVO_OPEN_ANGLE_RS   = 15;
const int SERVO_CLOSED_ANGLE_RS = 85;

// Function declarations
bool RunSelfAudit();
void HomeRobot();

#endif