#pragma once
#include "core.h"
#include <ArduinoJson.h>
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

struct PocketbaseState {
  bool isConnected = false;
  DynamicJsonDocument launcherRecord =
      DynamicJsonDocument(1024); // Adjust size as needed
  DynamicJsonDocument launchRecord =
      DynamicJsonDocument(1024); // Adjust size as needed
};