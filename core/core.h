struct FlightData {
  long unsigned timestamp;
  float pressure;
  float temperature;
  float altitude;
  float ax, ay, az;
  float gx, gy, gz;
  float batteryLevel;
  bool isDeployed;
  bool isFlying;
  bool isLanded;
};

int dataSendFrequency = 32; // 32 Hz
long dataSendDelay = 1000 / dataSendFrequency;

int ledBlinkFrequency = 4; // 4 Hz
long ledBlinkDelay = 1000 / ledBlinkFrequency;

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