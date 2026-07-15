// ================================================================
// ESP32 WiFi Bridge — UNO <-> Favoriot (two-way)
// ================================================================
// Upload:   reads status lines from the UNO on Serial2 and POSTs
//           them to Favoriot as data: { "status": "<msg>" }.
// Download: polls the device stream every POLL_INTERVAL_MS and
//           forwards data: { "command": "<value>" } entries to the
//           UNO (e.g. dashboard button sends command=CMD4).
//
// Dashboard button widget config:  Key = command   Value = CMD4
// (one button per command: CMD4 / CMD5 / CMD6 / STOP)
// ================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "../secrets.h"  // WiFi & Favoriot credentials (gitignored)

const char* favoriot_endpoint = FAVORIOT_API_URL;
const char* favoriot_apikey = FAVORIOT_API_KEY;
const char* device_developer_id = DEVICE_DEVELOPER_ID;

// Serial2 to the Arduino UNO: RX pin 16, TX pin 17
#define RXD2 16
#define TXD2 17

#define POLL_INTERVAL_MS 2000UL

WiFiMulti wifiMulti;
WiFiClientSecure secureClient;

unsigned long lastPollTime = 0;
String lastSeenStreamId = "";
bool baselineSet = false;  // first poll only records where the stream is,
                           // so a button pressed before boot is not replayed

void postStatusToFavoriot(String msg) {
  // The UNO sends some statuses wrapped in quotes — strip them so the
  // JSON we build here is always valid.
  msg.replace("\"", "");
  msg.trim();
  if (msg.length() == 0) return;

  HTTPClient http;
  http.begin(secureClient, favoriot_endpoint);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", favoriot_apikey);

  String jsonPayload = String("{\"device_developer_id\":\"") + device_developer_id
                     + "\",\"data\":{\"status\":\"" + msg + "\"}}";

  int httpResponseCode = http.POST(jsonPayload);
  http.end();

  if (httpResponseCode == 201 || httpResponseCode == 200) {
    Serial.printf("[Favoriot] Status '%s' sent OK (%d)\n", msg.c_str(), httpResponseCode);
  } else {
    Serial.printf("[Favoriot] Status '%s' FAILED (%d)\n", msg.c_str(), httpResponseCode);
  }
}

void pollCloudCommand() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastPollTime < POLL_INTERVAL_MS) return;
  lastPollTime = millis();

  HTTPClient http;
  String url = String("https://apiv2.favoriot.com/v2/devices/")
             + device_developer_id + "/streams?max=1&order=desc";

  http.begin(secureClient, url);
  http.addHeader("apikey", favoriot_apikey);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.GET();
  if (httpCode != 200) {
    Serial.printf("[Favoriot] Poll failed, HTTP %d\n", httpCode);
    http.end();
    return;
  }

  String payload = http.getString();
  http.end();

  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("[Favoriot] JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  JsonArray results = doc["results"].as<JsonArray>();
  if (results.isNull() || results.size() == 0) {
    baselineSet = true;  // empty stream is a valid baseline
    return;
  }

  JsonObject latest = results[0];
  String streamId = latest["stream_id"] | "";
  if (streamId.length() == 0 || streamId == lastSeenStreamId) return;

  // Mark as seen regardless of content, so the same entry is never
  // reacted to twice (status posts land in this stream too).
  lastSeenStreamId = streamId;
  if (!baselineSet) {
    baselineSet = true;
    return;
  }

  String command = latest["data"]["command"] | "";
  if (command.length() == 0) return;  // a status entry, not a button press

  Serial.printf("[Favoriot] Cloud command received: %s -> forwarding to UNO\n", command.c_str());
  Serial2.println(command);
}

void setup() {
  // Serial for USB debugging to your computer
  Serial.begin(115200);

  // Serial2 for communicating with the Arduino
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  wifiMulti.addAP(WIFI_SSID2, WIFI_PASSWORD2);

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

  secureClient.setInsecure();  // skip cert validation — fine for this project
}

void loop() {
  // UNO -> cloud: forward status lines
  if (Serial2.available()) {
    String incomingData = Serial2.readStringUntil('\n');
    incomingData.trim();
    if (incomingData.length() > 0) {
      Serial.print("Received from Arduino: ");
      Serial.println(incomingData);
      if (WiFi.status() == WL_CONNECTED) {
        postStatusToFavoriot(incomingData);
      }
    }
  }

  // Cloud -> UNO: forward dashboard button commands
  pollCloudCommand();
}
