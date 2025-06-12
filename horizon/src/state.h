#pragma once
#include "core.h"
#include <ArduinoJson.h>
#include <functional>
#include <stddef.h>
#include <stdint.h>

// Holds the state for flight data transfer
struct FlightDataState {
  FlightData flightDataReceived;
  FlightData currentFlightData = {0};
  FlightData previousFlightData = {0};
  uint8_t *flightDataBytes = (uint8_t *)&flightDataReceived;
  size_t flightDataOffset = 0;
  bool hasReceivedFlightData = false;
};

struct LoadingDataState {
  WaterLoadingData currentWaterLoadingData = {0};
  WaterLoadingData previousWaterLoadingData = {0};
  AirLoadingData currentAirLoadingData = {0};
  AirLoadingData previousAirLoadingData = {0};
  launchStates launchState = STAND_BY;
  bool hasReceivedLoadingData = false;
}; // This struct is simpler than FlightDataState because it doesn't need to
   // handle byte offsets or partial data.

struct PocketbaseState {
  bool isConnected = false;
  DynamicJsonDocument launcherRecord =
      DynamicJsonDocument(1024); // Adjust size as needed
  DynamicJsonDocument launchRecord =
      DynamicJsonDocument(1024); // Adjust size as needed
  DynamicJsonDocument rocketRecord =
      DynamicJsonDocument(1024); // Adjust size as needed
  std::function<void(PocketbaseState &)> launchControlCallback;
};