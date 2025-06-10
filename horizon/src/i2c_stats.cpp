#include "i2c_stats.h"
#include <Arduino.h>

void computeI2CDataRates(DataRateCounters &dataRateCounters) {
  if (millis() - dataRateCounters.lastRatePrint > 1000) {
    dataRateCounters.padLastRate = dataRateCounters.padBytesReceived;
    dataRateCounters.gatewayLastRate = dataRateCounters.gatewayBytesReceived;
    dataRateCounters.padBytesReceived = 0;
    dataRateCounters.gatewayBytesReceived = 0;
    dataRateCounters.lastRatePrint = millis();
  }
}

void printI2CDataRates(const DataRateCounters &dataRateCounters) {
  Serial.print("[I2C] Pad IN: ");
  Serial.print(dataRateCounters.padLastRate);
  Serial.print(" B/s, Gateway IN: ");
  Serial.print(dataRateCounters.gatewayLastRate);
  Serial.println(" B/s");
}
