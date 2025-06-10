#pragma once
#include <Arduino.h>

// Data rate counters for I2C IN
struct DataRateCounters {
  unsigned long padBytesReceived;
  unsigned long gatewayBytesReceived;
  unsigned long lastRatePrint;
  unsigned long padLastRate;
  unsigned long gatewayLastRate;
};

void computeI2CDataRates(DataRateCounters &dataRateCounters);
void printI2CDataRates(const DataRateCounters &dataRateCounters);
