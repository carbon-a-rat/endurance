#pragma once
#include <stddef.h>
#include <stdint.h>
#include "core.h"

// Holds the state for flight data transfer
struct FlightDataState {
  FlightData flightDataReceived;
  FlightData currentFlightData = {0};
  FlightData previousFlightData = {0};
  uint8_t *flightDataBytes = (uint8_t *)&flightDataReceived;
  size_t flightDataOffset = 0;
  bool hasReceivedFlightData = false;
};
