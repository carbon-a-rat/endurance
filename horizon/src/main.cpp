#include "PocketBaseClient.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <Timer.h>
#include <Wire.h>
#include <core.h> // For FlightData struct and dataSendFrequency

#define GATEWAY_I2C_ADDRESS 0x03

FlightData flightData;
uint8_t *flightDataBytes = (uint8_t *)&flightData;
size_t flightDataOffset = 0;

// WiFi credentials
const char *ssid = "L'antre.";
const char *password = "pannacotta";
// const char *ssid = "Fairphone 5";
// const char *password = "reptiliens";

// PocketBase server config
// const char *pb_host = "10.10.10.167";
const char *pb_host = "deathstar.cyp.sh";
uint16_t pb_port = 443; // Use 443 for HTTPS
const char *pb_identity = "death_star";
const char *pb_pass = "kF4X1hnaWQZYwy6";

PocketBaseClient pbClient(pb_host, pb_port);

// --- I2C Communication ---
void initI2C() {
  Wire.begin(4, 5); // SDA = GPIO4, SCL = GPIO5
  delay(1000);
  Serial.println("I2C Master ready.");
}

size_t requestFlightDataChunk() {
  size_t bytesToRequest = sizeof(FlightData) - flightDataOffset;
  if (bytesToRequest > 32)
    bytesToRequest = 32;

  Wire.requestFrom(GATEWAY_I2C_ADDRESS, (int)bytesToRequest);
  int bytesRead = Wire.readBytes((char *)(flightDataBytes + flightDataOffset),
                                 bytesToRequest);
  return bytesRead;
}

// --- Data Handling ---
void handleFlightDataChunk(int bytesRead) {
  if (bytesRead > 0) {
    flightDataOffset += bytesRead;
    if (flightDataOffset >= sizeof(FlightData)) {
      Serial.println("Received FlightData:");
      printFlightData(flightData);
      flightDataOffset = 0; // Ready for next packet
    }
  } else {
    Serial.println("No data received.");
  }
}

void onSSEMessage(const String &data) {
  Serial.print("[PocketBase SSE] ");
  Serial.println(data);
}

// Launcher data JSON initialization

String launcherJSON;
DynamicJsonDocument deserializedJSON(2048);

// --- Main Logic ---
void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Optional: set a different auth collection
  // pbClient.setAuthCollection("admins");
  pbClient.setAuthCollection("launchers");
  if (pbClient.login(pb_identity, pb_pass)) {
    Serial.println("PocketBase login successful!");
    launcherJSON = pbClient.getAuthRecordJson();
    Serial.println("Auth record: " + launcherJSON);
    // Start realtime connection and set callback
    pbClient.startRealtime(onSSEMessage);
    // Example: subscribe to a collection and a specific record
    deserializeJson(deserializedJSON, launcherJSON);
    String launcherId = deserializedJSON["id"].as<String>();
    pbClient.addRealtimeSubscription("lauchers", launcherId);
    // Subscriptions will be sent to the server automatically when clientId is
    // received
  } else {
    Serial.println("PocketBase login failed!");
  }

  initI2C();
}

void loop() {
  // Poll for SSE events
  pbClient.pollRealtime();

  static Timer pollTimer(250 / dataSendFrequency); // 4x frequency
  if (pollTimer.expired()) {
    int bytesRead = requestFlightDataChunk();
    handleFlightDataChunk(bytesRead);
    // Place for additional logic (e.g., error handling, state machines, etc.)
  }
}
