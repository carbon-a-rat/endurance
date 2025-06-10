#include "PocketBaseClient.h"
#include "i2c_stats.h"
#include "i2c_utils.h"
#include "state.h"
#include "wifi_utils.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Timer.h>
#include <WiFi.h>
#include <Wire.h>
#include <functional>

// Core Endurance library
#include <EnduranceConfig.h>
#include <core.h>

// Provide definitions for the global variables used in i2c_utils.cpp
FlightDataState flightDataState;
DataRateCounters dataRateCounters = {0, 0, 0, 0, 0};

PocketBaseClient pbClient(DEATH_STAR_POCKETBASE_HOST,
                          DEATH_STAR_POCKETBASE_PORT);

// --- Data Handling ---

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

// Callback for PocketBase SSE messages
void onSSEMessage(const String &data) {
  Serial.print("[PocketBase SSE] ");
  Serial.println(data);
}

// Launcher data JSON initialization

String launcherJSON;
JsonDocument launcher;
void setup() {
  Serial.begin(115200);
  initI2C();
  initWiFi([]() {}, // disconnectedCallback
           []() {}, // connectedCallback
           []() {}  // gotIPCallback
  );

  pbClient.setAuthCollection("launchers");

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

  Serial.println("Horizon ready");
}

void loop() {
  // Poll for SSE events
  pbClient.pollRealtime();

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
    printI2CDataRates(dataRateCounters);
  }

  debugSendCommandToGateway();
  printFlightData(flightDataState.currentFlightData);
}
