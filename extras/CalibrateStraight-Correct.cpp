#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.hpp>

// ================================================================
//  CalibrateStraight2.cpp
//  Simple straight-line calibration:
//    • Set the four wheel speeds by TYPING in the Serial Monitor.
//    • Press IR button 1 to drive all 4 wheels forward (stays driving).
//    • Press IR * (or type s) to stop.
//  Speeds are printed only when they change or when you start driving.
//  Adjust, drive, stop, reposition, repeat until it tracks straight.
// ================================================================

// ── Hardware ──────────────────────────────────────────────────
#define RECV_PIN  A3
mecanumCar car(3, 2);

// ── IR codes ──────────────────────────────────────────────────
#define CMD_1     0x16  // drive all 4 wheels forward
#define CMD_STAR  0x42  // stop

// ── The four library wheel-speed globals (this is what we tune) ─
extern uint8_t speed_Upper_L;  // front-left
extern uint8_t speed_Lower_L;  // rear-left
extern uint8_t speed_Upper_R;  // front-right
extern uint8_t speed_Lower_R;  // rear-right

bool driving = false;

void printSpeeds() {
  Serial.println(F("---- current wheel speeds ----"));
  Serial.print(F("  Front-Left  (speed_Upper_L) = ")); Serial.println(speed_Upper_L);
  Serial.print(F("  Front-Right (speed_Upper_R) = ")); Serial.println(speed_Upper_R);
  Serial.print(F("  Rear-Left   (speed_Lower_L) = ")); Serial.println(speed_Lower_L);
  Serial.print(F("  Rear-Right  (speed_Lower_R) = ")); Serial.println(speed_Lower_R);
}

void printHelp() {
  Serial.println(F("\n=== STRAIGHT-LINE CALIBRATION (Serial) ==="));
  Serial.println(F("Type in the Serial Monitor (newline ending):"));
  Serial.println(F("  one number   -> set ALL four wheels   (e.g.  80)"));
  Serial.println(F("  four numbers -> FL FR RL RR            (e.g.  80 82 78 79)"));
  Serial.println(F("  g            -> go (drive forward)"));
  Serial.println(F("  s            -> stop"));
  Serial.println(F("IR remote:  1 = drive forward,  * = stop"));
}

// Read a typed line and apply it.
void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("g")) { driving = true;  Serial.println(F(">> DRIVING")); return; }
  if (line.equalsIgnoreCase("s")) { driving = false; car.Stop(); Serial.println(F(">> STOPPED")); return; }
  if (line.equalsIgnoreCase("?") || line.equalsIgnoreCase("h")) { printHelp(); return; }

  // Parse up to four numbers separated by spaces or commas.
  char buf[40];
  line.toCharArray(buf, sizeof(buf));
  int vals[4];
  int n = 0;
  for (char *tok = strtok(buf, " ,"); tok && n < 4; tok = strtok(NULL, " ,")) {
    vals[n++] = constrain(atoi(tok), 0, 255);
  }

  if (n == 1) {                       // one number -> all wheels
    speed_Upper_L = speed_Upper_R = speed_Lower_L = speed_Lower_R = vals[0];
  } else if (n == 4) {                // four numbers -> FL FR RL RR
    speed_Upper_L = vals[0];          // FL
    speed_Upper_R = vals[1];          // FR
    speed_Lower_L = vals[2];          // RL
    speed_Lower_R = vals[3];          // RR
  } else {
    Serial.println(F("?? type 1 number (all) or 4 numbers (FL FR RL RR). 'h' for help."));
    return;
  }
  printSpeeds();                      // print on change
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }

  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  printHelp();
  printSpeeds();
  Serial.println(F("Place robot on the floor, set speeds, press 1 to drive."));
}

void loop() {
  handleSerial();

  if (IrReceiver.decode()) {
    uint8_t key = IrReceiver.decodedIRData.command;
    IrReceiver.resume();
    if (key == CMD_1) {
      driving = true;
      Serial.println(F(">> DRIVING (button 1)"));
      printSpeeds();                  // print on start
    } else if (key == CMD_STAR) {
      driving = false;
      car.Stop();
      Serial.println(F(">> STOPPED"));
    }
  }

  // Re-assert every pass (guards a dropped I2C frame). Stays driving until stopped.
  if (driving) {
    car.Advance();          // reads the four speed_* globals we set above
  } else {
    car.Stop();
  }
  delay(20);
}
