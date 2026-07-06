#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include "../secrets.h"  // WiFi & Favoriot credentials (gitignored)

// ==========================================
// 1. WI-FI & FAVORIOT CONFIGURATION
// ==========================================
// All sensitive values live in secrets.h (not committed to git)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* ssid2 = WIFI_SSID2;
const char* password2 = WIFI_PASSWORD2;
WiFiMulti wifiMulti;
const char* favoriot_endpoint = FAVORIOT_API_URL;
const char* favoriot_apikey = FAVORIOT_API_KEY;
const char* device_developer_id = DEVICE_DEVELOPER_ID;

// We will use Hardware Serial 2 to talk to the Arduino
// RX pin 16, TX pin 17
#define RXD2 16
#define TXD2 17

void setup() {
  // Serial for USB debugging to your computer
  Serial.begin(115200);
  
  // Serial2 for communicating with the Arduino
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println();
  Serial.println("Scanning WiFi networks...");
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    Serial.printf("  %2d: %s (RSSI %d) ch%d %s\n", i + 1,
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "secured");
  }
  if (n == 0) Serial.println("  (no networks found)");

  wifiMulti.addAP(ssid, password);
  wifiMulti.addAP(ssid2, password2);

  Serial.print("Connecting to WiFi...");
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("Network: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Check if the Arduino has sent any data
  if (Serial2.available()) {
    // Read the incoming data as a string until a newline character
    String incomingData = Serial2.readStringUntil('\n');
    incomingData.trim(); // Remove any extra spaces or hidden characters
    
    if (incomingData.length() > 0) {
      Serial.print("Received from Arduino: ");
      Serial.println(incomingData);

      // Send to Favoriot if WiFi is connected
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(favoriot_endpoint);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("apikey", favoriot_apikey);
        
        // Build JSON payload using the data from Arduino
        String jsonPayload = "{\"device_developer_id\":\"" + String(device_developer_id) + "\",\"data\":{\"distance\":" + incomingData + "}}";
        
        int httpResponseCode = http.POST(jsonPayload);
        
        String result;
        if (httpResponseCode == 201 || httpResponseCode == 200) {
          result = "Favoriot OK: " + String(httpResponseCode);
        } else {
          result = "Favoriot FAIL: " + String(httpResponseCode);
        }
        Serial.println(result);
        Serial2.println(result);  // relay to Uno so it shows in the USB serial monitor
        http.end();
      }
    }
  }
}