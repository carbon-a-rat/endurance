#include "i2c_utils.h"
#include "EnduranceConfig.h"
#include "core.h"
#include "i2c_stats.h"
#include "state.h"
#include <Arduino.h>
#include <Wire.h>

// I2C pin definitions
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

extern FlightDataState flightDataState;

// Recovers the I2C bus by toggling the clock and data lines
void i2cBusRecovery() {
  pinMode(SDA_PIN, OUTPUT); // SDA
  pinMode(SCL_PIN, OUTPUT); // SCL
  digitalWrite(SDA_PIN, HIGH);
  for (int i = 0; i < 9; ++i) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
}

// Initializes the I2C bus and prints status
void initI2C() {
  i2cBusRecovery();
  Wire.begin();
  Wire.setClock(ENDURANCE_I2C_BUS_SPEED); // Set I2C bus speed
  Wire.setTimeOut(ENDURANCE_I2C_TIMEOUT);
  Serial.print("I2C Master ready at ");
  Serial.print(ENDURANCE_I2C_BUS_SPEED);
  Serial.println(" Hz.");
}

// Workaround for ESP32-S2 I2C frequency drop issue
void i2cWorkaround() {
  Wire.setClock(ENDURANCE_I2C_BUS_SPEED_WORKAROUND);
  Wire.setClock(ENDURANCE_I2C_BUS_SPEED); // Restore original speed
}

// Handles a chunk of flight data received from the gateway
void handleFlightDataChunk(uint8_t *chunk, int bytesRead,
                           FlightDataState &state) {
  if (bytesRead == 0) {
    state.flightDataOffset = 0; // Reset offset
    return;
  }
  if (bytesRead < 2) {
    Serial.println("Received invalid flight data chunk, too small.");
    return;
  }
  uint8_t offset = chunk[0];
  size_t dataLen = bytesRead - 1;
  if (offset + dataLen > sizeof(FlightData)) {
    dataLen = sizeof(FlightData) - offset;
  }
  memcpy(state.flightDataBytes + offset, chunk + 1, dataLen);
  state.flightDataOffset += dataLen;
  if (state.flightDataOffset >= sizeof(FlightData)) {
    if (!state.hasReceivedFlightData) {
      state.hasReceivedFlightData = true;
      return; // Ignore first packet as it's probably corrupted
    }
    state.previousFlightData = state.currentFlightData;
    state.currentFlightData = state.flightDataReceived;
    state.flightDataOffset = 0; // Ready for next packet
  }
}

// Requests a chunk of flight data from the gateway over I2C
size_t requestFlightDataChunk(DataRateCounters &counter,
                              FlightDataState &state) {
  uint8_t chunk[GATEWAY_I2C_CHUNK_SIZE];
  size_t bytesRequested = sizeof(chunk);
  Wire.requestFrom(GATEWAY_I2C_ADDRESS, (int)bytesRequested);
  int bytesRead = Wire.readBytes((char *)chunk, bytesRequested);
#ifdef DEBUG
  for (int i = 0; i < bytesRead; ++i) {
    Serial.print(chunk[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif
  bool allZero = true, allFF = true;
  for (int i = 0; i < bytesRead; ++i) {
    if (chunk[i] != 0)
      allZero = false;
    if (chunk[i] != 0xFF)
      allFF = false;
  }
  if (allZero || allFF) {
    handleFlightDataChunk(chunk, 0, state); // No data available
    counter.gatewayBytesReceived += 1;
    return 0;
  }
  counter.gatewayBytesReceived += bytesRead;
  handleFlightDataChunk(chunk, bytesRead, state);
  return bytesRead;
}

// Handles a chunk of pad data received from the pad
void handlePadDataChunk(uint8_t *chunk, int bytesRead, FlightDataState &state) {
#ifdef DEBUG
  Serial.print("[PAD I2C] Received ");
  Serial.print(bytesRead);
  Serial.println(" bytes from pad:");
  for (int i = 0; i < bytesRead; ++i) {
    Serial.print("0x");
    if (chunk[i] < 16)
      Serial.print("0");
    Serial.print(chunk[i], HEX);
    Serial.print(" ");
    if ((i + 1) % 16 == 0)
      Serial.println();
  }
  Serial.println();
#endif
}

// Requests a chunk of pad data from the pad over I2C
size_t requestPadDataChunk(DataRateCounters &counter, FlightDataState &state) {
  uint8_t chunk[PAD_I2C_CHUNK_SIZE];
  size_t bytesRequested = sizeof(chunk);
  Wire.requestFrom(PAD_I2C_ADDRESS, (int)bytesRequested);
  int bytesAvailable = Wire.available();
  int bytesRead = Wire.readBytes((char *)chunk, bytesRequested);
  counter.padBytesReceived += bytesRead;
  handlePadDataChunk(chunk, bytesRead, state);
  return bytesRead;
}

// Sends a command string to the gateway over I2C
void sendCommandToGateway(const String &command) {
  Serial.print("Sending command to gateway: ");
  Serial.println(command);
  Wire.beginTransmission(GATEWAY_I2C_ADDRESS);
  Wire.write((const uint8_t *)command.c_str(), command.length());
  Wire.endTransmission();
}
