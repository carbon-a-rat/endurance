#pragma once
#include "i2c_stats.h"
#include "state.h"
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

void i2cBusRecovery();
void initI2C();
void i2cWorkaround();
size_t requestFlightDataChunk(DataRateCounters &counter,
                              FlightDataState &state);
void handleFlightDataChunk(uint8_t *chunk, int bytesRead,
                           FlightDataState &state);
size_t requestPadDataChunk(DataRateCounters &counter, LoadingDataState &state);
void handlePadDataChunk(uint8_t *chunk, int bytesRead, LoadingDataState &state);
void sendCommandToGateway(const String &command);
