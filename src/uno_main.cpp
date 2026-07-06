#include <Arduino.h>
#include <SoftwareSerial.h>

// Ultrasonic sensor (HC-SR04)
const uint8_t ULTRASONIC_TRIG_PIN = 12;
const uint8_t ULTRASONIC_ECHO_PIN = 13;

// Link to ESP32: Uno pin 10 (TX) -> ESP32 RX2, Uno pin 11 (RX) <- ESP32 TX2
SoftwareSerial espSerial(11, 10);  // RX, TX

const unsigned long SEND_INTERVAL_MS = 5000;

uint16_t readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long echoUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000UL);
  if (echoUs == 0) return 0;  // timeout / out of range
  return echoUs / 58;
}

void setup() {
  Serial.begin(9600);       // USB debug
  espSerial.begin(9600);    // link to ESP32

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  Serial.println("Uno: ultrasonic -> ESP32 sender ready");
}

void loop() {
  static unsigned long lastSendMs = 0;

  if (millis() - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = millis();

    uint16_t distanceCm = readDistanceCm();
    if (distanceCm > 0) {
      espSerial.println(distanceCm);   // number only, e.g. "25\n"
      Serial.print("Sent distance: ");
      Serial.print(distanceCm);
      Serial.println(" cm");
    } else {
      Serial.println("No echo (out of range), skipped");
    }
  }

  // Show anything the ESP32 sends back (status messages)
  while (espSerial.available()) {
    Serial.write(espSerial.read());
  }
}
