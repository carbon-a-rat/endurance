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

void printPadDataPacket(const PadDataPacket &packet) {
  Serial.print("PadDataPacket Type: ");
  switch (packet.type) {
  case PadDataPacket::NO_DATA:
    Serial.println("NO_DATA");
    break;
  case PadDataPacket::WATER_LOADING_DATA:
    Serial.println("WATER_LOADING_DATA");
    printWaterLoadingData(packet.data.waterLoadingData);
    break;
  case PadDataPacket::AIR_LOADING_DATA:
    Serial.println("AIR_LOADING_DATA");
    printAirLoadingData(packet.data.airLoadingData);
    break;
  default:
    Serial.println("UNKNOWN_TYPE");
    break;
  }
  Serial.print("Launch State: ");
  Serial.println(packet.launchState);
}

// Serialize PadDataPacket into a byte buffer
uint8_t *serializePadDataPacket(const PadDataPacket &packet, size_t &size) {
  // Calculate size based on type
  size = sizeof(packet.type) + sizeof(packet.launchState);

  switch (packet.type) {
  case PadDataPacket::NO_DATA:
    // No additional data to serialize
    break;

  case PadDataPacket::WATER_LOADING_DATA:
    size += sizeof(WaterLoadingData);
    break;

  case PadDataPacket::AIR_LOADING_DATA:
    size += sizeof(AirLoadingData);
    break;

  default:
    // Handle unknown type
    break;
  }

  // Allocate buffer
  uint8_t *buffer = new uint8_t[size];
  uint8_t *current = buffer;

  // Serialize type
  memcpy(current, &packet.type, sizeof(packet.type));
  current += sizeof(packet.type);

  // Serialize launchState
  memcpy(current, &packet.launchState, sizeof(packet.launchState));
  // Serial.print("Serialized launchState bytes: ");
  // for (size_t i = 0; i < sizeof(packet.launchState); ++i) {
  //     Serial.print(((uint8_t *)&packet.launchState)[i], HEX);
  //     Serial.print(" ");
  // }
  // Serial.println();
  current += sizeof(packet.launchState);

  // Serialize data based on type
  switch (packet.type) {
  case PadDataPacket::NO_DATA:
    // No additional data to serialize
    break;

  case PadDataPacket::WATER_LOADING_DATA:
    memcpy(current, &packet.data.waterLoadingData, sizeof(WaterLoadingData));
    break;

  case PadDataPacket::AIR_LOADING_DATA:
    memcpy(current, &packet.data.airLoadingData, sizeof(AirLoadingData));
    break;

  default:
    // Handle unknown type
    break;
  }

  return buffer;
}

// Deserialize PadDataPacket from a byte buffer
PadDataPacket deserializePadDataPacket(const uint8_t *buffer,
                                       size_t bufferSize) {
  PadDataPacket packet;

  if (bufferSize < sizeof(packet.type) + sizeof(packet.launchState)) {
    // Handle error: buffer size is too small
    memset(&packet, 0, sizeof(PadDataPacket));
    return packet;
  }

  // Deserialize type
  memcpy(&packet.type, buffer, sizeof(packet.type));
  buffer += sizeof(packet.type);
  bufferSize -= sizeof(packet.type);

  // Deserialize launchState
  memcpy(&packet.launchState, buffer, sizeof(packet.launchState));
  // Serial.print("Deserialized launchState bytes: ");
  // for (size_t i = 0; i < sizeof(packet.launchState); ++i) {
  //     Serial.print(((uint8_t *)&packet.launchState)[i], HEX);
  //     Serial.print(" ");
  // }
  // Serial.println();
  buffer += sizeof(packet.launchState);
  bufferSize -= sizeof(packet.launchState);

  // Deserialize data based on type
  switch (packet.type) {
  case PadDataPacket::NO_DATA:
    // No additional data to deserialize
    memset(&packet.data, 0, sizeof(packet.data));
    break;

  case PadDataPacket::WATER_LOADING_DATA:
    if (bufferSize >= sizeof(WaterLoadingData)) {
      memcpy(&packet.data.waterLoadingData, buffer, sizeof(WaterLoadingData));
    } else {
      // Handle error: buffer size is too small for WaterLoadingData
      memset(&packet.data.waterLoadingData, 0, sizeof(WaterLoadingData));
    }
    break;

  case PadDataPacket::AIR_LOADING_DATA:
    if (bufferSize >= sizeof(AirLoadingData)) {
      memcpy(&packet.data.airLoadingData, buffer, sizeof(AirLoadingData));
    } else {
      // Handle error: buffer size is too small for AirLoadingData
      memset(&packet.data.airLoadingData, 0, sizeof(AirLoadingData));
    }
    break;

  default:
    // Handle unknown type
    memset(&packet.data, 0, sizeof(packet.data));
    break;
  }

  return packet;
}
