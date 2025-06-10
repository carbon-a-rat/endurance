#pragma once
#include "core.h"
#include <stddef.h>

// Holds the state for flight data transfer
struct FlightDataState {
  FlightData flightDataReceived;
  FlightData currentFlightData = {0};
  FlightData previousFlightData = {0};
  uint8_t *flightDataBytes = (uint8_t *)&flightDataReceived;
  size_t flightDataOffset = 0;
  bool hasReceivedFlightData = false;
};

// Data rate counters for I2C IN
struct DataRateCounters {
  unsigned long padBytesReceived;
  unsigned long gatewayBytesReceived;
  unsigned long lastRatePrint;
  unsigned long padLastRate;
  unsigned long gatewayLastRate;
};
