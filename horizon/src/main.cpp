#include "i2c_stats.h"
#include "i2c_utils.h"
#include "ntp_utils.h"
#include "pocketbase_utils.h"
#include "state.h"
#include "wifi_utils.h"
#include <Arduino.h>
#include <ArduinoJson.h>
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
LoadingDataState loadingDataState; // Holds the state for loading data
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

void debugSendCommandToPad() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      sendCommandToPad(command);
    }
  }
}

void setup() {
  Serial.begin(115200);
  initI2C();
  initWiFi([]() {},                     // disconnectedCallback
           []() {},                     // connectedCallback
           []() { initNtp(ntpClient); } // gotIPCallback
  );
  // initPocketbase(pocketbaseConnection, pocketbaseState);
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
    // requestPadDataChunk(dataRateCounters, loadingDataState);
  }

  computeI2CDataRates(dataRateCounters);

  if (dataRateTimer.expired()) {
    // printI2CDataRates(dataRateCounters);
    printFlightData(flightDataState.currentFlightData);
    // printAirLoadingData(loadingDataState.currentAirLoadingData);
    // printWaterLoadingData(loadingDataState.currentWaterLoadingData);
  }

  ntpClient.update(); // Update NTP time
  // pocketbaseLoop(pocketbaseState, pocketbaseConnection);

  // debugSendCommandToGateway();
  debugSendCommandToPad();
}
