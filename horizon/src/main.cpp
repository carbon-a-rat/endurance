#include "PocketBaseClient.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Timer.h>
#include <WiFi.h>
#include <Wire.h>

// Core Endurance library
#include <EnduranceConfig.h>
#include <core.h>

FlightData flightData;
uint8_t *flightDataBytes = (uint8_t *)&flightData;
size_t flightDataOffset = 0;
bool hasReceivedFlightData = false;

PocketBaseClient pbClient(DEATH_STAR_POCKETBASE_HOST,
                          DEATH_STAR_POCKETBASE_PORT);

// --- I2C Communication ---
void initI2C() {
  Wire.begin(); // ESP32: You can use default pins (SDA=21, SCL=22) or set

  delay(1000);
  Serial.println("I2C Master ready.");
}

void handleFlightDataChunk(uint8_t *chunk, int bytesRead);

size_t requestFlightDataChunk() {
  uint8_t chunk[32];
  size_t bytesRequested = sizeof(chunk);
  Wire.requestFrom(GATEWAY_I2C_ADDRESS, (int)bytesRequested);
  int bytesAvailable = Wire.available();
  int bytesRead = Wire.readBytes((char *)chunk, bytesRequested);
  handleFlightDataChunk(chunk, bytesRead);
  return bytesRead;
}

// --- Data Handling ---
void handleFlightDataChunk(uint8_t *chunk, int bytesRead) {
  // Detect a full zero packet (no data available from gateway)
  bool allZero = true;
  for (int i = 0; i < bytesRead; ++i) {
    if (chunk[i] != 0) {
      allZero = false;
      break;
    }
  }
  if (allZero) {
    // No data received, reset offset
    flightDataOffset = 0;
    return;
  }
  if (bytesRead < 2) {
    Serial.println("Error: Not enough bytes received from I2C slave.");
    Serial.println("No data received.");
    return;
  }
  uint8_t offset = chunk[0];
  size_t dataLen = bytesRead - 1;
  if (offset + dataLen > sizeof(FlightData)) {
    dataLen = sizeof(FlightData) - offset;
  }
  memcpy(flightDataBytes + offset, chunk + 1, dataLen);
  flightDataOffset += dataLen;
  if (flightDataOffset >= sizeof(FlightData)) {
    if (!hasReceivedFlightData) {
      hasReceivedFlightData = true;
      return; // Ignore first packet has its probably corrupted
    }
    Serial.println("Received FlightData:");
    printFlightData(flightData);
    flightDataOffset = 0; // Ready for next packet
  }
}

void sendCommandToGateway(const String &command) {
  Serial.print("Sending command to gateway: ");
  Serial.println(command);
  Wire.beginTransmission(GATEWAY_I2C_ADDRESS);
  Wire.write((const uint8_t *)command.c_str(), command.length());
  Wire.endTransmission();
}

void debugSendCommandToGateway() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      sendCommandToGateway(command);
    }
  }
}

void onSSEMessage(const String &data) {
  Serial.print("[PocketBase SSE] ");
  Serial.println(data);
}

// Launcher data JSON initialization

String launcherJSON;
JsonDocument launcher;

// --- Main Logic ---
void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");

  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 30000;

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startAttemptTime < wifiTimeout) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed!");
    Serial.print("WiFi status: ");
    if (WiFi.status() == WL_NO_SSID_AVAIL) {
      Serial.println("No SSID available");
    } else if (WiFi.status() == WL_CONNECT_FAILED) {
      Serial.println("Connection failed");
    } else if (WiFi.status() == WL_DISCONNECTED) {
      Serial.println("Disconnected");
    } else {
      Serial.println("Unknown status: " + String(WiFi.status()));
    }
    Serial.println("Restarting in 2 seconds...");
    delay(2000);
    ESP.restart();
    return;
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  pbClient.setAuthCollection("launchers");
  if (pbClient.login(DEATH_STAR_POCKETBASE_IDENTITY,
                     DEATH_STAR_POCKETBASE_PASSWORD)) {
    Serial.println("PocketBase login successful!");
    launcherJSON = pbClient.getAuthRecordJson();
    Serial.println("Auth record: " + launcherJSON);
    // Start realtime connection and set callback
    pbClient.startRealtime(onSSEMessage);
    Serial.println("Establishing realtime connection...");
    // while (!pbClient.isRealtimeConnected()) {
    //   pbClient.pollRealtime();
    // }
    Serial.println("Realtime connection started!");

    deserializeJson(launcher, launcherJSON);
    String launcherId = launcher["id"].as<String>();
    pbClient.addRealtimeSubscription("launchers", launcherId);
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

  static Timer pollTimer(DATA_SEND_INTERVAL / 4); // Polls 4 times more often
                                                  // than data send frequency
  if (pollTimer.expired()) {
    int bytesRead = requestFlightDataChunk();
    // Place for additional logic (e.g., error handling, state machines, etc.)
  }

  debugSendCommandToGateway();
}
