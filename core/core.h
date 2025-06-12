// Public constants and structures for the whole endurance project
#pragma once

#define ENDURANCE_CORE_H

#define GATEWAY_DEBUG
#define PAD_DEBUG

const int DATA_SEND_FREQUENCY = 32; // 32 Hz
const long unsigned DATA_SEND_INTERVAL = 1000 / DATA_SEND_FREQUENCY;

const int PAD_DATA_SEND_FREQUENCY = 8; // 8 Hz
const long unsigned PAD_DATA_SEND_INTERVAL = 1000 / PAD_DATA_SEND_FREQUENCY;

const int LED_BLINK_FREQUENCY = 4; // 4 Hz
const long unsigned LED_BLINK_DELAY = 1000 / LED_BLINK_FREQUENCY;

// I2C addresses for gateway and pad
const long ENDURANCE_I2C_BUS_SPEED = 400000L; // 400 kHz fast mode
const long ENDURANCE_I2C_BUS_SPEED_WORKAROUND =
    1000000L; // 1 MHz workaround for ESP32
const int ENDURANCE_I2C_TIMEOUT = 25;

const int GATEWAY_I2C_CHUNK_SIZE = 64; // bytes per I2C transfer
const char GATEWAY_I2C_ADDRESS = 0x03;

const char PAD_I2C_ADDRESS = 0x04;
const int PAD_I2C_CHUNK_SIZE = 32; // bytes per I2C transfer

// Probe deployment and landing constants
const int PROBE_DEPLOYMENT_TIME_THRESHOLD = 300;
const int PROBE_SERVO_INITIAL_POSITION = 0;
const int PROBE_SERVO_DEPLOYED_POSITION = 180;
const float PROBE_DEPLOYMENT_ALTITUDE_DROP_AFTER_APOGEE = 0.5f; // meters
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

#pragma pack(push, 1)
struct WaterLoadingData {
  long unsigned timestamp;
  float waterVolume;
  float waterFlowRate;
  float error;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AirLoadingData {
  long unsigned timestamp;
  float pressure;
  float error;
};
#pragma pack(pop)

enum launchStates {
  STAND_BY,
  FILLING_WATER,
  FILLED_WATER,
  FILLING_AIR,
  READY_TO_LAUNCH,
  LAUNCHING,
  LAUNCHED,
  CANCELLING,
  CANCELLED,
};

#pragma pack(push, 1)
struct PadDataPacket {
  enum { NO_DATA, WATER_LOADING_DATA, AIR_LOADING_DATA } type;
  launchStates launchState;
  union {
    WaterLoadingData waterLoadingData;
    AirLoadingData airLoadingData;
  } data;
};
#pragma pack(pop)

void printFlightData(const FlightData &flightData);
void printWaterLoadingData(const WaterLoadingData &data);
void printAirLoadingData(const AirLoadingData &data);