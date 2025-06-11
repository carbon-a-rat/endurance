#include "i2c_stats.h"
#include "i2c_utils.h"
#include "pocketbase.hpp"
#include "state.h"
#include "wifi_utils.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <Timer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <functional>

// Core Endurance library
#include <EnduranceConfig.h>
#include <core.h>

// Global state structures
FlightDataState flightDataState;
DataRateCounters dataRateCounters = {0, 0, 0, 0, 0};
PocketbaseState pocketbaseState;

// NTP client setup
WiFiUDP ntpUdp;
NTPClient ntpClient(ntpUdp, "pool.ntp.org", 0,
                    60000); // NTP client for time synchronization

// Pocketbase connection
PocketbaseArduino pocketbaseConnection(DEATH_STAR_POCKETBASE_HOST);

// Debug utils

// Reads a command from Serial and sends it to the gateway
void debugSendCommandToGateway() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      sendCommandToGateway(command);
    }
  }
}

void initNtp() {
  ntpClient.begin();  // Start NTP client after WiFi is connected
  ntpClient.update(); // Update time immediately
  Serial.print("NTP Time: ");
  Serial.print(ntpClient.getFormattedTime());
  Serial.print(", Epoch: ");
  Serial.println(ntpClient.getEpochTime());
}

void onLauncherUpdate(SubscriptionEvent &ev, void *ctx);

void initPocketbase() {
  bool connected = false;
  int retryCount = 0;
  const int maxRetries = 10;
  Serial.println("Connecting to Pocketbase...");
  while (!connected && retryCount < maxRetries) {
    connected = pocketbaseConnection.login_passwd(
        DEATH_STAR_POCKETBASE_IDENTITY, DEATH_STAR_POCKETBASE_PASSWORD,
        "launchers");
    Serial.print("Pocketbase login attempt: ");
    Serial.println(connected);
    if (!connected) {
      Serial.println("Pocketbase login failed, retrying in 2s...");
      delay(2000);
      retryCount++;
    } else {
      Serial.println("Pocketbase login successful!");
      pocketbaseState.launcherRecord =
          pocketbaseConnection.getConnectionRecord();
      pocketbaseState.isConnected = true;
      Serial.println("Pocketbase launcher record: " +
                     pocketbaseState.launcherRecord.as<String>());
      pocketbaseConnection.subscribe(
          "launchers",
          pocketbaseState.launcherRecord["record"]["id"].as<String>().c_str(),
          onLauncherUpdate, &pocketbaseState);
    }
  }
  if (!connected) {
    Serial.println("ERROR: Could not connect to Pocketbase after retries.");
  }
}

void onLauncherUpdate(SubscriptionEvent &ev, void *ctx) {
  // Handle launcher updates here
  PocketbaseState *state = static_cast<PocketbaseState *>(ctx);
  if (ev.valid && ev.event == "update") {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, ev.data);
    if (!error) {
      state->launcherRecord = doc;
      Serial.println("Launcher record updated: " +
                     state->launcherRecord.as<String>());
      // Optionally, you can trigger other actions based on the update
    } else {
      Serial.printf("Failed to parse launcher update: %s\n", error.c_str());
    }
  }
}

String epochToIso(time_t epochTime) {
  struct tm *timeinfo = gmtime(&epochTime);
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", timeinfo);
  return String(buffer);
}

void heartbeat() {
  // Send a heartbeat to the Pocketbase server
  if (pocketbaseState.isConnected) {
    pocketbaseConnection.update(
        "launchers",
        pocketbaseState.launcherRecord["record"]["id"].as<String>().c_str(),
        "{\"last_ping_at\": \"" + epochToIso(ntpClient.getEpochTime()) +
            "\", \"online\": true}"); // Now formatted as ISO 8601 UTC
    Serial.println("Heartbeat sent to Pocketbase.");
  } else {
    Serial.println("Pocketbase not connected, skipping heartbeat.");
  }
}

void pocketbaseLoop() {
  static Timer heartbeatTimer(5000); // 5 seconds heartbeat interval
  if (heartbeatTimer.expired()) {
    heartbeat();
  }
  if (pocketbaseState.isConnected) {
    pocketbaseConnection.update_subscription();
  }
}

void printFreeHeap() {
  Serial.print("Free heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.print(" bytes, ");
  Serial.print("Min free heap: ");
  Serial.println(ESP.getMinFreeHeap());
}

void setup() {
  Serial.begin(115200);
  initI2C();
  initWiFi([]() {},            // disconnectedCallback
           []() {},            // connectedCallback
           []() { initNtp(); } // gotIPCallback
  );
  initPocketbase();
  Serial.println("Horizon ready");
}

void loop() {
  static Timer gatewayTimer(DATA_SEND_INTERVAL / 4); // Gateway poll timer
  static Timer padTimer(DATA_SEND_INTERVAL / 2);     // Pad poll timer
  static Timer dataRateTimer(1000);                  // Data rate print timer

  i2cWorkaround(); // Ensure I2C bus is functional

  if (gatewayTimer.expired()) {
    requestFlightDataChunk(dataRateCounters, flightDataState);
  }
  if (padTimer.expired()) {
    requestPadDataChunk(dataRateCounters, flightDataState);
  }

  computeI2CDataRates(dataRateCounters);

  if (dataRateTimer.expired()) {
    // printI2CDataRates(dataRateCounters);
    // printFlightData(flightDataState.currentFlightData);
  }

  ntpClient.update(); // Update NTP time
  pocketbaseLoop();   // Handle Pocketbase subscriptions

  debugSendCommandToGateway();
}
