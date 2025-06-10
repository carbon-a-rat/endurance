#include "i2c_stats.h"
#include "i2c_utils.h"
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

// Provide definitions for the global variables used in i2c_utils.cpp
FlightDataState flightDataState;
DataRateCounters dataRateCounters = {0, 0, 0, 0, 0};

WiFiUDP ntpUdp;
NTPClient ntpClient(ntpUdp, "pool.ntp.org", 0,
                    60000); // NTP client for time synchronization

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

void setup() {
  Serial.begin(115200);
  initI2C();
  initWiFi([]() {}, // disconnectedCallback
           []() {}, // connectedCallback
           []() {
             ntpClient.begin();  // Start NTP client after WiFi is connected
             ntpClient.update(); // Update time immediately
             Serial.print("NTP Time: ");
             Serial.print(ntpClient.getFormattedTime());
             Serial.print(", Epoch: ");
             Serial.println(ntpClient.getEpochTime());
           } // gotIPCallback
  );

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
    printI2CDataRates(dataRateCounters);
    printFlightData(flightDataState.currentFlightData);
  }

  debugSendCommandToGateway();
}
