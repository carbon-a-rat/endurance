#include <Arduino.h>
#include <Timer.h>
#include <Wire.h>
#include <core.h> // For FlightData struct and dataSendFrequency

#define GATEWAY_I2C_ADDRESS 0x03

FlightData flightData;
uint8_t *flightDataBytes = (uint8_t *)&flightData;
size_t flightDataOffset = 0;

// --- I2C Communication ---
void initI2C() {
  Wire.begin(4, 5); // SDA = GPIO4, SCL = GPIO5
  delay(1000);
  Serial.println("I2C Master ready.");
}

size_t requestFlightDataChunk() {
  size_t bytesToRequest = sizeof(FlightData) - flightDataOffset;
  if (bytesToRequest > 32)
    bytesToRequest = 32;

  Wire.requestFrom(GATEWAY_I2C_ADDRESS, (int)bytesToRequest);
  int bytesRead = Wire.readBytes((char *)(flightDataBytes + flightDataOffset),
                                 bytesToRequest);
  return bytesRead;
}

// --- Data Handling ---
void handleFlightDataChunk(int bytesRead) {
  if (bytesRead > 0) {
    flightDataOffset += bytesRead;
    if (flightDataOffset >= sizeof(FlightData)) {
      Serial.println("Received FlightData:");
      printFlightData(flightData);
      flightDataOffset = 0; // Ready for next packet
    }
  } else {
    Serial.println("No data received.");
  }
}

// --- Main Logic ---
void setup() {
  Serial.begin(115200);
  initI2C();
}

void loop() {
  static Timer pollTimer(250 / dataSendFrequency); // 4x frequency

  if (pollTimer.expired()) {
    int bytesRead = requestFlightDataChunk();
    handleFlightDataChunk(bytesRead);
    // Place for additional logic (e.g., error handling, state machines, etc.)
  }
}
