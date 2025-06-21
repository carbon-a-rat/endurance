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

void launchControlCallback(PocketbaseState &pocketbaseState) {
  Serial.println("Launch control callback triggered.");
  if (loadingDataState.launchState == STAND_BY &&
      pocketbaseState.launchRecord["record"]["should_load"].as<bool>()) {
    float waterVolumicPercentage =
        pocketbaseState.launchRecord["record"]["water_volumic_percentage"]
            .as<float>();
    Serial.print("Water volumic percentage: ");
    Serial.println(waterVolumicPercentage);
    float rocketVolume;
    String serializedRocketRecord =
        pocketbaseConnection.collection("rockets").getOne(
            pocketbaseState.launchRecord["record"]["rocket"]
                .as<String>()
                .c_str(),
            nullptr, "volume");
    DynamicJsonDocument rocketDoc(1024);
    DeserializationError error =
        deserializeJson(rocketDoc, serializedRocketRecord);
    if (!error) {
      pocketbaseState.rocketRecord = rocketDoc;
      rocketVolume = pocketbaseState.rocketRecord["volume"].as<float>();
      Serial.println(pocketbaseState.rocketRecord.as<String>());
      Serial.print("Rocket volume: ");
      Serial.println(rocketVolume);
    } else {
      Serial.println("Failed to deserialize rocket record JSON");
      rocketVolume = 0.0;
    }
    String command =
        "fill_water " + String(waterVolumicPercentage * rocketVolume, 2);
    sendCommandToPad(command);
    Serial.println("Water loading command sent: " + command);
  } else if (loadingDataState.launchState == FILLED_WATER &&
             pocketbaseState.launchRecord["record"]["should_load"].as<bool>()) {
    float airPressure =
        pocketbaseState.launchRecord["record"]["pressure"].as<float>();
    String command = "fill_air " + String(airPressure, 2);
    sendCommandToPad(command);
    Serial.println("Air loading command sent: " + command);
  } else if (loadingDataState.launchState == READY_TO_LAUNCH &&
             pocketbaseState.launchRecord["record"]["should_launch"]
                 .as<bool>()) {
    String command = "launch";
    sendCommandToPad(command);
    sendCommandToGateway("GO");
    Serial.println("Launch command sent");
  } else if (loadingDataState.launchState != STAND_BY &&
             pocketbaseState.launchRecord["record"]["should_cancel"]
                 .as<bool>()) {
    String command = "cancel";
    sendCommandToPad(command);
    sendCommandToGateway("RESET");
    Serial.println("Launch cancel command sent");
  } else {
    Serial.println("No action needed for launch control.");
  }
}

struct PocketbaseHeartbeatTaskParams {
  PocketbaseState &pocketbaseState;
  PocketbaseArduino &pocketbaseConnection;
  NTPClient &ntpClient;
};

PocketbaseHeartbeatTaskParams pocketbaseHeartbeatTaskParams = {
    pocketbaseState, pocketbaseConnection, ntpClient};

void pocketBaseHeartbeatTask(void *pvParameters) {
  PocketbaseHeartbeatTaskParams *params =
      static_cast<PocketbaseHeartbeatTaskParams *>(pvParameters);
  if (params->pocketbaseState.isConnected) {
    heartbeatPocketbase(params->pocketbaseConnection, params->pocketbaseState,
                        params->ntpClient);

    vTaskDelay(5000 / portTICK_PERIOD_MS); // Heartbeat every 5 seconds
  }
}

void setupTasks() {
  xTaskCreatePinnedToCore(pocketBaseHeartbeatTask,        // Task function
                          "PocketbaseHeartbeat",          // Task name
                          2048,                           // Stack size
                          &pocketbaseHeartbeatTaskParams, // Task parameters
                          1,                              // Priority
                          NULL,                           // Task handle
                          1);                             // Core ID
}

void setup() {
  Serial.begin(115200);
  initI2C();
  /*initWiFi([]() {},                     // disconnectedCallback
           []() {},                     // connectedCallback
           []() { initNtp(ntpClient); } // gotIPCallback
  );
  */
  pocketbaseState.launchControlCallback = launchControlCallback;
  // initPocketbase(pocketbaseConnection, pocketbaseState);
  // setupTasks(); // Initialize tasks
  Serial.println("Horizon ready");
}

void loop() {
  static Timer gatewayTimer(DATA_SEND_INTERVAL / 4); // Gateway poll timer
  static Timer padTimer(PAD_DATA_SEND_INTERVAL / 2); // Pad poll timer
  static Timer dataRateTimer(1000);                  // Data rate print timer

  i2cWorkaround(); // Ensure I2C bus is functional

  if (gatewayTimer.expired()) {
    requestFlightDataChunk(dataRateCounters, flightDataState);
  }
  if (padTimer.expired()) {
    requestPadDataChunk(dataRateCounters, loadingDataState);
  }

  computeI2CDataRates(dataRateCounters);

  if (dataRateTimer.expired()) {
    printI2CDataRates(dataRateCounters);
    // printFlightData(flightDataState.currentFlightData);
    printAirLoadingData(loadingDataState.currentAirLoadingData);
    printWaterLoadingData(loadingDataState.currentWaterLoadingData);
    String launchState;
    switch (loadingDataState.launchState) // Print launch state
    {
    case STAND_BY:
      launchState = "Standby";
      break;
    case FILLING_WATER:
      launchState = "Filling Water";
      break;
    case FILLED_WATER:
      launchState = "Filled Water";
      break;
    case FILLING_AIR:
      launchState = "Filling Air";
      break;
    case READY_TO_LAUNCH:
      launchState = "Ready to Launch";
      break;
    case LAUNCHED:
      launchState = "Launched";
      break;
    case CANCELLING:
      launchState = "Cancelling";
      break;
    case LAUNCHING:
      launchState = "Launching";
      break;
    case CANCELLED:
      launchState = "Cancelled";
      break;
    default:
      launchState = "Invalid State: " + String(loadingDataState.launchState);
      break;
    }
    Serial.print("Launch State: ");
    Serial.println(launchState);
  }

  // ntpClient.update(); // Update NTP time
  //  pocketbaseLoop(pocketbaseState, pocketbaseConnection, ntpClient);

  // debugSendCommandToGateway();
  debugSendCommandToPad();
}
