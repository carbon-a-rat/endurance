#pragma once
#include <stddef.h>
#include <stdint.h>

void i2cBusRecovery();
void initI2C();
void i2cWorkaround();
size_t requestFlightDataChunk();
void handleFlightDataChunk(uint8_t *chunk, int bytesRead);
size_t requestPadDataChunk();
void handlePadDataChunk(uint8_t *chunk, int bytesRead);
void sendCommandToGateway(const String &command);
