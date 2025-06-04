// Public constants and structures for the whole endurance project
#pragma once
const int DATA_SEND_FREQUENCY = 32; // 32 Hz
const long unsigned DATA_SEND_INTERVAL = 1000 / DATA_SEND_FREQUENCY;

const int LED_BLINK_FREQUENCY = 4; // 4 Hz
const long unsigned LED_BLINK_DELAY = 1000 / LED_BLINK_FREQUENCY;

// I2C addresses for gateway and pad
const int I2C_BUS_SPEED = 100000;      // 400 kHz fast mode
const int GATEWAY_I2C_CHUNK_SIZE = 32; // bytes per I2C transfer

const char GATEWAY_I2C_ADDRESS = 0x03;
const char PAD_I2C_ADDRESS = 0x04;

// Probe deployment and landing constants
const int PROBE_DEPLOYMENT_TIME_THRESHOLD = 300;
const int PROBE_SERVO_INITIAL_POSITION = 0;
const int PROBE_SERVO_DEPLOYED_POSITION = 180;
const float PROBE_DEPLOYMENT_ALTITUDE_DROP_AFTER_APOGEE = 0.2f; // meters
const float PROBE_LANDING_ALTITUDE_STABLE_WINDOW = 0.3f;        // meters
const long PROBE_LANDING_ALTITUDE_STABLE_TIME =
    2000; // ms, how long altitude must be stable

#pragma pack(push, 1)
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
#pragma pack(pop)

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