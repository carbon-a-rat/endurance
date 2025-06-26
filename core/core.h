// Public constants and structures for the whole endurance project
#pragma once

#define ENDURANCE_CORE_H

// #define GATEWAY_DEBUG
//  #define PAD_DEBUG

#include <Arduino.h>

const int DATA_SEND_FREQUENCY = 16; // 16 Hz
const long unsigned DATA_SEND_INTERVAL = 1000 / DATA_SEND_FREQUENCY;

const int PAD_DATA_SEND_FREQUENCY = 8; // 8 Hz
const long unsigned PAD_DATA_SEND_INTERVAL = 1000 / PAD_DATA_SEND_FREQUENCY;

const int LED_BLINK_FREQUENCY = 4; // 4 Hz
const long unsigned LED_BLINK_DELAY = 1000 / LED_BLINK_FREQUENCY;

// I2C addresses for gateway and pad
const long ENDURANCE_I2C_BUS_SPEED = 50000; // 100 kHz fast mode
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

  // Comparison operators
  bool operator==(const FlightData &other) const {
    return timestamp == other.timestamp && pressure == other.pressure &&
           temperature == other.temperature && altitude == other.altitude &&
           ax == other.ax && ay == other.ay && az == other.az &&
           gx == other.gx && gy == other.gy && gz == other.gz &&
           batteryLevel == other.batteryLevel &&
           isDeployed == other.isDeployed && isFlying == other.isFlying &&
           isLanded == other.isLanded;
  }

  bool operator!=(const FlightData &other) const { return !(*this == other); }
};
#pragma pack(pop)

#pragma pack(push, 1)
struct WaterLoadingData {
  long unsigned timestamp;
  float waterVolume;
  float waterFlowRate;
  float error;

  // Comparison operators
  bool operator==(const WaterLoadingData &other) const {
    return timestamp == other.timestamp && waterVolume == other.waterVolume &&
           waterFlowRate == other.waterFlowRate && error == other.error;
  }

  bool operator!=(const WaterLoadingData &other) const {
    return !(*this == other);
  }
};
#pragma pack(pop)

#pragma pack(push, 1)
struct AirLoadingData {
  long unsigned timestamp;
  float pressure;
  float error;

  // Comparison operators
  bool operator==(const AirLoadingData &other) const {
    return timestamp == other.timestamp && pressure == other.pressure &&
           error == other.error;
  }

  bool operator!=(const AirLoadingData &other) const {
    return !(*this == other);
  }
};
#pragma pack(pop)

struct LauncherData {
  bool isProbeConnected;
  uint8_t probeBatteryLevel; // 0-100%
  float lastPressure;
  float lastWaterFlow;
  float lastAcc;
  float lastAltitude;
};

enum launchStates : uint8_t {
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
  enum : uint8_t { NO_DATA, WATER_LOADING_DATA, AIR_LOADING_DATA } type;
  launchStates launchState;
  union {
    WaterLoadingData waterLoadingData;
    AirLoadingData airLoadingData;
  } data;
};
#pragma pack(pop)

enum LaunchUpdateType : uint8_t {
  STARTED_WATER_LOADING,
  STARTED_AIR_LOADING,
  LOAD_COMPLETED,
  FIRED,
  LANDED,
  CANCELLED_,
};

struct LaunchUpdate {
  LaunchUpdateType type;
  long unsigned timestamp;
};

enum LogLevel : uint8_t { LOG_DEBUG, LOG_INFO, LOG_WARNING, LOG_ERROR };

struct LauncherLog {
  long unsigned timestamp;
  LogLevel level;
  String event;
  String message;
};

struct Heartbeat {
  long unsigned timestamp;
};

void printFlightData(const FlightData &flightData);
void printWaterLoadingData(const WaterLoadingData &data);
void printAirLoadingData(const AirLoadingData &data);
void printPadDataPacket(const PadDataPacket &packet);
uint8_t *serializePadDataPacket(const PadDataPacket &packet, size_t &size);
PadDataPacket deserializePadDataPacket(const uint8_t *buffer,
                                       size_t bufferSize);