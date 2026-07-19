#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include "../secrets.h"  // WiFi & Favoriot credentials (gitignored)

// ================================================================
// WIFI / FAVORIOT CREDENTIALS
// WIFI_SSID2/WIFI_PASSWORD2 (training-room network), FAVORIOT_API_KEY,
// DEVICE_DEVELOPER_ID, MQTT_PORT and DEVICE_ACCESS_TOKEN (device access
// token, NOT the API key) all come from secrets.h at the project root.
// ================================================================

// ================================================================
// FAVORIOT HTTP STREAM
// ================================================================
const char* FAVORIOT_ENDPOINT =
    "https://apiv2.favoriot.com/v2/streams";

// ================================================================
// FAVORIOT MQTT RPC
// ================================================================
const char* MQTT_HOST = "mqtt.favoriot.com";

// ================================================================
// ESP32 <-> ARDUINO UNO UART
// ESP32 GPIO16 RX2 <- UNO D10 TX
// ESP32 GPIO17 TX2 -> UNO D11 RX
// Common GND required
// ================================================================
#define RXD2 16
#define TXD2 17

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

String rpcTopic;

// ================================================================
// WIFI CONNECTION
// ================================================================
void connectWiFi()
{
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }

    Serial.println();
    Serial.print("Connecting to WiFi");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID2, WIFI_PASSWORD2);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
}

// ================================================================
// MQTT RPC CALLBACK
// ================================================================
void mqttCallback(
    char* topic,
    byte* payload,
    unsigned int length
)
{
    String message;

    for (unsigned int i = 0; i < length; i++) {
        message += static_cast<char>(payload[i]);
    }

    message.trim();
    message.toUpperCase();

    Serial.println();
    Serial.print("MQTT topic: ");
    Serial.println(topic);

    Serial.print("RPC message: ");
    Serial.println(message);

    // Favoriot buttons may send JSON such as:
    // {"COMMAND":"CMD4"}
    // {"COMMAND":"CMD5"}
    // {"COMMAND":"CMD6"}
    //
    // Plain CMD4, CMD5, and CMD6 are also accepted.
    if (message.indexOf("CMD4") >= 0) {
        Serial.println("Blue button received.");
        Serial.println("Sending CMD4 to Arduino UNO...");

        Serial2.println("CMD4");
    }
    else if (message.indexOf("CMD5") >= 0) {
        Serial.println("Red button received.");
        Serial.println("Sending CMD5 to Arduino UNO...");

        Serial2.println("CMD5");
    }
    else if (message.indexOf("CMD6") >= 0) {
        Serial.println("Yellow button received.");
        Serial.println("Sending CMD6 to Arduino UNO...");

        Serial2.println("CMD6");
    }
    else if (
        message.indexOf("STAR") >= 0 ||
        message.indexOf("CMDSTAR") >= 0 ||
        message.indexOf("STOP") >= 0 ||
        message.indexOf("EMERGENCY_STOP") >= 0
    ) {
        Serial.println("Emergency STOP received.");
        Serial.println("Sending STOP to Arduino UNO...");
        Serial2.println("STOP");
    }
    else {
        Serial.println("Unknown RPC command.");
    }
}

// ================================================================
// MQTT CONNECTION
// ================================================================
void connectMQTT()
{
    if (mqttClient.connected()) {
        return;
    }

    while (
        WiFi.status() == WL_CONNECTED &&
        !mqttClient.connected()
    ) {
        Serial.print("Connecting to Favoriot MQTT...");

        String clientId = "RobotESP32-";
        clientId += String(
            static_cast<uint32_t>(ESP.getEfuseMac()),
            HEX
        );

        bool connected = mqttClient.connect(
            clientId.c_str(),
            DEVICE_ACCESS_TOKEN,
            DEVICE_ACCESS_TOKEN
        );

        if (connected) {
            Serial.println("connected!");

            bool subscribed =
                mqttClient.subscribe(rpcTopic.c_str(), 1);

            if (subscribed) {
                Serial.print("Subscribed to RPC topic: ");
                Serial.println(rpcTopic);
            } else {
                Serial.println("RPC subscribe failed.");
            }
        }
        else {
            Serial.print("failed. MQTT state: ");
            Serial.println(mqttClient.state());

            delay(3000);
        }
    }
}

// ================================================================
// UPLOAD ULTRASONIC DISTANCE TO FAVORIOT
// ================================================================
void uploadDistance(float distanceVal)
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(
            "WiFi disconnected. Distance upload skipped."
        );
        return;
    }

    HTTPClient http;

    http.begin(FAVORIOT_ENDPOINT);
    http.addHeader(
        "Content-Type",
        "application/json"
    );
    http.addHeader(
        "apikey",
        FAVORIOT_API_KEY
    );

    String jsonPayload =
        "{\"device_developer_id\":\"" +
        String(DEVICE_DEVELOPER_ID) +
        "\",\"data\":{\"distance\":" +
        String(distanceVal, 2) +
        "}}";

    Serial.print("Payload sending: ");
    Serial.println(jsonPayload);

    int responseCode = http.POST(jsonPayload);

    if (
        responseCode == 200 ||
        responseCode == 201
    ) {
        Serial.println(
            "SUCCESS: Data accepted by Favoriot!"
        );
    }
    else if (responseCode > 0) {
        Serial.print("Favoriot HTTP error: ");
        Serial.println(responseCode);

        Serial.print("Server response: ");
        Serial.println(http.getString());
    }
    else {
        Serial.print("HTTP connection error: ");
        Serial.println(
            http.errorToString(responseCode)
        );
    }

    http.end();
}

// ================================================================
// UPLOAD MISSION STATUS TO FAVORIOT
// The UNO sends statuses wrapped in quotes, e.g. "task4_complete".
// ================================================================
void uploadStatus(String statusMsg)
{
    statusMsg.replace("\"", "");
    statusMsg.trim();

    if (statusMsg.length() == 0) {
        return;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(
            "WiFi disconnected. Status upload skipped."
        );
        return;
    }

    String jsonPayload =
        "{\"device_developer_id\":\"" +
        String(DEVICE_DEVELOPER_ID) +
        "\",\"data\":{\"status\":\"" +
        statusMsg +
        "\"}}";

    Serial.print("Status payload sending: ");
    Serial.println(jsonPayload);

    // One retry so a transient HTTP failure does not lose the status.
    for (int attempt = 1; attempt <= 2; attempt++) {
        HTTPClient http;

        http.begin(FAVORIOT_ENDPOINT);
        http.addHeader(
            "Content-Type",
            "application/json"
        );
        http.addHeader(
            "apikey",
            FAVORIOT_API_KEY
        );

        int responseCode = http.POST(jsonPayload);

        if (
            responseCode == 200 ||
            responseCode == 201
        ) {
            Serial.println(
                "SUCCESS: Status accepted by Favoriot!"
            );
            http.end();
            return;
        }

        Serial.print("Status upload attempt ");
        Serial.print(attempt);
        Serial.print(" failed: ");

        if (responseCode > 0) {
            Serial.println(responseCode);
            Serial.print("Server response: ");
            Serial.println(http.getString());
        } else {
            Serial.println(
                http.errorToString(responseCode)
            );
        }

        http.end();
        delay(500);
    }
}

// ================================================================
// PROCESS DATA RECEIVED FROM UNO
// ================================================================
void processUnoSerial()
{
    if (!Serial2.available()) {
        return;
    }

    String incomingData =
        Serial2.readStringUntil('\n');

    incomingData.replace("\r", "");
    incomingData.trim();

    if (incomingData.length() == 0) {
        return;
    }

    Serial.print("UNO message: [");
    Serial.print(incomingData);
    Serial.println("]");

    bool validNumber = true;
    bool decimalFound = false;

    for (
        unsigned int i = 0;
        i < incomingData.length();
        i++
    ) {
        char c = incomingData.charAt(i);

        if (isDigit(c)) {
            continue;
        }

        if (c == '.' && !decimalFound) {
            decimalFound = true;
            continue;
        }

        validNumber = false;
        break;
    }

    if (validNumber) {
        float distanceVal =
            incomingData.toFloat();

        Serial.print("Valid distance: ");
        Serial.println(distanceVal, 2);

        uploadDistance(distanceVal);
        return;
    }

    // FullTest1 sends UNO_READY during startup.
    if (incomingData == "UNO_READY") {
        Serial.println(
            "Arduino UNO connection confirmed."
        );
    }

    // Any other text from the UNO is a mission status
    // (e.g. "task4_complete") — upload it to Favoriot.
    uploadStatus(incomingData);
}

// ================================================================
// SETUP
// ================================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println(
        "ESP32 Favoriot MQTT + HTTP V1"
    );

    Serial2.begin(
        9600,
        SERIAL_8N1,
        RXD2,
        TXD2
    );

    Serial2.setTimeout(50);

    rpcTopic =
        String(DEVICE_ACCESS_TOKEN) +
        "/v2/rpc";

    connectWiFi();

    mqttClient.setServer(
        MQTT_HOST,
        MQTT_PORT
    );

    mqttClient.setCallback(
        mqttCallback
    );

    mqttClient.setBufferSize(1024);

    connectMQTT();

    Serial.println(
        "ESP32 cloud gateway ready."
    );
}

// ================================================================
// LOOP
// ================================================================
void loop()
{
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }

    if (!mqttClient.connected()) {
        connectMQTT();
    }

    // Must run repeatedly to receive Favoriot button RPC.
    mqttClient.loop();

    processUnoSerial();

    delay(5);
}