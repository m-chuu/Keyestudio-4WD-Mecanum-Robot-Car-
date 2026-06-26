/*
  ================================================================
  RobotSequences.cpp  (fileB)
  ----------------------------------------------------------------
  Reuses the per-button action functions saved in ButtonFunctions.cpp
  (fileA) to build 3 pre-programmed sequences, one per remote button.

  Building blocks borrowed from fileA:
    moveForwardBlocks(blocks, speed)  -> drive forward over N black lines
    rotateLeft90()  / rotateRight90() -> 90-degree pivot (sensor stop)
    spinRight180()                    -> ~180-degree turn in place

  Sequences:
    Button 1 ->  forward 6 lines, turn 180, forward 6 lines
    Button 2 ->  forward 4 lines, turn LEFT 90, forward 2,
                 turn RIGHT 90, forward 2 lines
    Button 3 ->  forward 4 lines, turn RIGHT 90, forward 2,
                 turn LEFT 90, forward 2 lines

  NOTE: fileA holds all the includes, pin defines, IR codes, speed
  defines and the `car` object, so we simply include it here. These
  files live in /extras and are NOT part of the PlatformIO src build,
  so there is no double-compilation.
  ================================================================
*/

#include "ButtonFunctions.cpp"   // fileA: brings in all button functions + globals

// ================================================================
//  SEQUENCE 1
//  Forward 6 black lines -> turn 180 -> forward 6 black lines
// ================================================================
void runSequence1() {
  Serial.println(F("\n=== SEQUENCE 1: fwd 6, turn 180, fwd 6 ==="));

  moveForwardBlocks(6, SPEED_START_FAST);   // forward 6 black lines

  if (!spinRight180()) return;              // turn 180 (abort on E-STOP)

  moveForwardBlocks(6, SPEED_START_FAST);   // forward 6 black lines

  Serial.println(F("=== SEQUENCE 1 complete ==="));
}

// ================================================================
//  SEQUENCE 2
//  Forward 4 -> turn LEFT 90 -> forward 2 -> turn RIGHT 90 -> forward 2
// ================================================================
void runSequence2() {
  Serial.println(F("\n=== SEQUENCE 2: fwd 4, L90, fwd 2, R90, fwd 2 ==="));

  moveForwardBlocks(4, SPEED_START_FAST);   // forward 4 black lines

  if (!rotateLeft90()) return;              // turn left 90 (abort on E-STOP)
  moveForwardBlocks(2, SPEED_START_SLOW);   // forward 2 black lines

  if (!rotateRight90()) return;             // turn right 90 (abort on E-STOP)
  moveForwardBlocks(2, SPEED_START_SLOW);   // forward 2 black lines

  Serial.println(F("=== SEQUENCE 2 complete ==="));
}

// ================================================================
//  SEQUENCE 3
//  Forward 4 -> turn RIGHT 90 -> forward 2 -> turn LEFT 90 -> forward 2
// ================================================================
void runSequence3() {
  Serial.println(F("\n=== SEQUENCE 3: fwd 4, R90, fwd 2, L90, fwd 2 ==="));

  moveForwardBlocks(4, SPEED_START_FAST);   // forward 4 black lines

  if (!rotateRight90()) return;             // turn right 90 (abort on E-STOP)
  moveForwardBlocks(2, SPEED_START_SLOW);   // forward 2 black lines

  if (!rotateLeft90()) return;              // turn left 90 (abort on E-STOP)
  moveForwardBlocks(2, SPEED_START_SLOW);   // forward 2 black lines

  Serial.println(F("=== SEQUENCE 3 complete ==="));
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);

  pinMode(SENSOR_LEFT,  INPUT);
  pinMode(SENSOR_MID,   INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  car.Init();
  IrReceiver.begin(RECV_PIN, false);

  Serial.println(F("Ready (Sequence mode)."));
  Serial.println(F("Press '1': forward 6, turn 180, forward 6."));
  Serial.println(F("Press '2': forward 4, left 90, forward 2, right 90, forward 2."));
  Serial.println(F("Press '3': forward 4, right 90, forward 2, left 90, forward 2."));
}

void loop() {
  if (!IrReceiver.decode()) return;

  uint8_t cmd = IrReceiver.decodedIRData.command;

  if (cmd == 0x00 || (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
    IrReceiver.resume();
    return;
  }

  if (cmd == CMD_1) {
    runSequence1();
  } else if (cmd == CMD_2) {
    runSequence2();
  } else if (cmd == CMD_3) {
    runSequence3();
  }

  IrReceiver.resume();
}
