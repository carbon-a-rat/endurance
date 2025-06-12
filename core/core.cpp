#include "core.h"
#include <Arduino.h>

void printFlightData(const FlightData &flightData) {
  Serial.print("Timestamp: ");
  Serial.print(flightData.timestamp);
  Serial.print(" ms, Battery Level: ");
  Serial.print(flightData.batteryLevel);
  Serial.print("%, ");
  Serial.print("Pressure: ");
  Serial.print(flightData.pressure);
  Serial.print(" hPa, Temperature: ");
  Serial.print(flightData.temperature);
  Serial.print(" °C, Altitude: ");
  Serial.print(flightData.altitude);
  Serial.print(" m, Accelerometer: ");
  Serial.print(flightData.ax);
  Serial.print(", ");
  Serial.print(flightData.ay);
  Serial.print(", ");
  Serial.print(flightData.az);
  Serial.print(" Gyroscope: ");
  Serial.print(flightData.gx);
  Serial.print(", ");
  Serial.print(flightData.gy);
  Serial.print(", ");
  Serial.print(flightData.gz);
  Serial.print(" dps");
  Serial.print("Is Deployed: ");
  Serial.print(flightData.isDeployed);
  Serial.print(", Is Flying: ");
  Serial.print(flightData.isFlying);
  Serial.print(", Is Landed: ");
  Serial.print(flightData.isLanded);
  Serial.println();
}

void printWaterLoadingData(const WaterLoadingData &data) {
  Serial.print("Timestamp: ");
  Serial.print(data.timestamp);
  Serial.print(" ms, Water Volume: ");
  Serial.print(data.waterVolume);
  Serial.print(" L, Water Flow Rate: ");
  Serial.print(data.waterFlowRate);
  Serial.print(" L/s, Error: ");
  Serial.print(data.error);
  Serial.println(" L");
}
void printAirLoadingData(const AirLoadingData &data) {
  Serial.print("Timestamp: ");
  Serial.print(data.timestamp);
  Serial.print(" ms, Pressure: ");
  Serial.print(data.pressure);
  Serial.print(" kPa, Error: ");
  Serial.print(data.error);
  Serial.println(" kPa");
}
