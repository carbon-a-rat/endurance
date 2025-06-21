#include <Arduino.h>
#include <Timer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

// Core Endurance library
#include <EnduranceConfig.h>
#include <core.h>

struct GatewayState {
  uint16_t packetQueueSize;
  uint16_t packetQueueMaxSize;
  uint16_t currentPacketIndex;
  uint8_t **packetQueue;
};

// The maximum duration (in seconds) for which the gateway will store data
// packets before data loss occurs.
int8_t dataQueueSizeSeconds = 10;

GatewayState gatewayState = {
    0, (uint16_t)(dataQueueSizeSeconds *DATA_SEND_FREQUENCY), 0, NULL};

bool ledState = false;
long unsigned lastLedChangeTime;

// --- Data rate monitoring ---
unsigned long i2cBytesSentTotal = 0;
unsigned long i2cBytesReceivedTotal = 0; // NEW: Track I2C incoming
unsigned long espNowBytesSentTotal = 0;
unsigned long espNowBytesReceivedTotal = 0; // NEW: Track ESP-NOW incoming
void printDataRates();

// --- Add this struct for the second queue ---
struct OutgoingQueue {
  uint8_t **packetQueue;
  uint16_t *packetSizes;
  uint16_t packetQueueSize;
  uint16_t packetQueueMaxSize;
  uint16_t currentPacketIndex;
};

#ifdef GATEWAY_DEBUG
bool debugIsFlying = false;
bool debugIsDeployed = false;
bool debugIsLanded = false;
unsigned long debugFlightStartTime = 0;
unsigned long debugDeploymentTime = 0;
unsigned long debugLandingTime = 0;
#endif

int OUTGOING_QUEUE_MAX_SIZE = 16;

OutgoingQueue outgoingQueue = {NULL, NULL, 0, OUTGOING_QUEUE_MAX_SIZE, 0};

void macAddressToByteArray(const char *macAddress, uint8_t *byteArray) {
  int i = 0;
  char *macAddressCopy = strdup(macAddress);
  char *token = strtok(macAddressCopy, ":");
  while (token != NULL && i < 6) {
    byteArray[i] = (uint8_t)strtol(token, NULL, 16);
    token = strtok(NULL, ":");
    i++;
  }
}

uint8_t probeMacAddress[6];

void printMacAddresses() {
  Serial.print("Probe MAC Address:  ");
  Serial.println(PROBE_MAC_ADDRESS);
  Serial.print("Gateway MAC Address:  ");
  Serial.println(WiFi.macAddress());
}

// Initialize the packet queue in setup()
void initializeGatewayQueue() {
  gatewayState.packetQueue =
      (uint8_t **)calloc(gatewayState.packetQueueMaxSize, sizeof(uint8_t *));
  if (!gatewayState.packetQueue) {
    Serial.println("Failed to allocate gateway packet queue!");
    while (true)
      delay(1000);
  }
}

// --- Initialize the outgoing queue ---
void initializeOutgoingQueue() {
  outgoingQueue.packetQueue =
      (uint8_t **)calloc(outgoingQueue.packetQueueMaxSize, sizeof(uint8_t *));
  outgoingQueue.packetSizes = (uint16_t *)calloc(
      outgoingQueue.packetQueueMaxSize, sizeof(uint16_t)); // <-- Add this
  if (!outgoingQueue.packetQueue || !outgoingQueue.packetSizes) {
    Serial.println("Failed to allocate outgoing packet queue!");
    while (true)
      delay(1000);
  }
}

// Add a packet to the queue
void enqueueGatewayPacket(uint8_t *packet, size_t packetSize) {
  uint16_t newPacketIndex =
      (gatewayState.currentPacketIndex + gatewayState.packetQueueSize) %
      gatewayState.packetQueueMaxSize;
  if (gatewayState.packetQueueSize < gatewayState.packetQueueMaxSize) {
    gatewayState.packetQueue[newPacketIndex] = packet; // Just store the pointer
    gatewayState.packetQueueSize++;
  } else {
    // Serial.println("Gateway packet queue is full, discarding packet.");
    free(packet); // Free the packet if not enqueued
  }
}

// --- Enqueue function for outgoing queue ---
void enqueueOutgoingPacket(uint8_t *packet, size_t packetSize) {
  uint16_t newPacketIndex =
      (outgoingQueue.currentPacketIndex + outgoingQueue.packetQueueSize) %
      outgoingQueue.packetQueueMaxSize;
  if (outgoingQueue.packetQueueSize < outgoingQueue.packetQueueMaxSize) {
    outgoingQueue.packetQueue[newPacketIndex] = packet;
    outgoingQueue.packetSizes[newPacketIndex] = packetSize; // <-- Store size
    outgoingQueue.packetQueueSize++;
  } else {
    Serial.println("Outgoing packet queue is full, discarding packet.");
    free(packet);
  }
}

// Remove and free the first packet in the queue
void dequeueGatewayPacket() {
  if (gatewayState.packetQueueSize > 0) {
    free(gatewayState.packetQueue[gatewayState.currentPacketIndex]);
    gatewayState.packetQueue[gatewayState.currentPacketIndex] = NULL;
    gatewayState.currentPacketIndex =
        (gatewayState.currentPacketIndex + 1) % gatewayState.packetQueueMaxSize;
    gatewayState.packetQueueSize--;
  }
}

// --- Dequeue function for outgoing queue ---
void dequeueOutgoingPacket() {
  if (outgoingQueue.packetQueueSize > 0) {
    free(outgoingQueue.packetQueue[outgoingQueue.currentPacketIndex]);
    outgoingQueue.packetQueue[outgoingQueue.currentPacketIndex] = NULL;
    outgoingQueue.packetSizes[outgoingQueue.currentPacketIndex] =
        0; // <-- Clear size
    outgoingQueue.currentPacketIndex = (outgoingQueue.currentPacketIndex + 1) %
                                       outgoingQueue.packetQueueMaxSize;
    outgoingQueue.packetQueueSize--;
  }
}

void onDataReceived(unsigned char *mac_addr, unsigned char *data, uint8_t len) {
  if (len < sizeof(FlightData)) {
    Serial.println("Received data too short for FlightData!");
    return;
  }
  FlightData flightData;
  memset(&flightData, 0, sizeof(FlightData));
  memcpy(&flightData, data, sizeof(FlightData));
  printFlightData(flightData);

  uint8_t *packet = (uint8_t *)malloc(len);
  if (packet) {
    memcpy(packet, data, len);
    enqueueGatewayPacket(packet, len);
  } else {
    Serial.println("Failed to allocate memory for received packet.");
  }

  espNowBytesReceivedTotal += len;

  // No integrated LED blinking on esp32-pico
  /*if (millis() - lastLedChangeTime > LED_BLINK_DELAY) {
    if (ledState) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    ledState = !ledState;
    lastLedChangeTime = millis();
  } */
}

// Add these globals for chunking
uint16_t i2cSendOffset = 0;
uint16_t i2cSendPacketSize = 0;
unsigned long lastI2CActivity = 0; // Track last I2C activity time

// Modify onI2CRequest to send up to 32 bytes at a time
void onI2CRequest() {
  lastI2CActivity = millis();
  if (gatewayState.packetQueueSize > 0) {
    uint8_t *packet = gatewayState.packetQueue[gatewayState.currentPacketIndex];
    size_t packetSize = sizeof(FlightData);

    // Calculate how many bytes to send this time (max GATEWAY_I2C_CHUNK_SIZE-1
    // for data)
    size_t bytesLeft = packetSize - i2cSendOffset;
    size_t chunkSize = bytesLeft > (GATEWAY_I2C_CHUNK_SIZE - 1)
                           ? (GATEWAY_I2C_CHUNK_SIZE - 1)
                           : bytesLeft;

    uint8_t chunk[GATEWAY_I2C_CHUNK_SIZE];
    chunk[0] = (uint8_t)i2cSendOffset; // Offset header
    memcpy(chunk + 1, packet + i2cSendOffset, chunkSize);

    Wire.write(chunk, chunkSize + 1);
    i2cBytesSentTotal += (chunkSize + 1); // <-- Count bytes sent over I2C
    i2cSendOffset += chunkSize;

    if (i2cSendOffset >= packetSize) {
      // Finished sending this packet
      dequeueGatewayPacket();
      i2cSendOffset = 0;
    }
  } else {
    // No packet available, send a 1-byte status code (0xFF = no data)
    uint8_t empty = 0xFF;
    Wire.write(&empty, 1);
    i2cBytesSentTotal += 1;
    i2cSendOffset = 0;
  }
}

void onI2CReceive(int numBytes) {
  lastI2CActivity = millis();
  if (numBytes > 0) {
    i2cBytesReceivedTotal += numBytes; // NEW: Count incoming I2C bytes
    uint8_t *packet = (uint8_t *)malloc(numBytes);
    if (packet) {
      for (int i = 0; i < numBytes; ++i) {
        if (Wire.available()) {
          packet[i] = Wire.read();
        }
      }
      enqueueOutgoingPacket(packet, numBytes);
      Serial.print("Enqueued packet for probe via ESP-NOW, size: ");
      Serial.println(numBytes);
#ifdef GATEWAY_DEBUG
      String command = String((char *)packet);
      Serial.print("Received I2C command: ");
      Serial.println(command);
      if (command == "GO") {
        debugIsFlying = true;
        debugFlightStartTime = millis();

      } else if (command == "RESET") {
        debugIsFlying = false;
        debugIsDeployed = false;
        debugIsLanded = false;
      }
#endif
    } else {
      Serial.println("Failed to allocate memory for outgoing packet.");
      // Drain the buffer
      while (Wire.available())
        Wire.read();
    }
  }
}

void initializeEspNow() {
  macAddressToByteArray(PROBE_MAC_ADDRESS, probeMacAddress); // Use global
  Serial.println("ESP-NOW initialized successfully.");

  // ESP32: Register receive callback
  esp_now_register_recv_cb([](const uint8_t *mac_addr, const uint8_t *data,
                              int data_len) {
    onDataReceived((unsigned char *)mac_addr, (unsigned char *)data, data_len);
  });

  // ESP32: Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, probeMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(probeMacAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer!");
    }
  }
}

void initI2CSlave() {
  Wire.end(); // In case already initialized
  delay(10);
  Wire.begin(GATEWAY_I2C_ADDRESS, 21, 22,
             ENDURANCE_I2C_BUS_SPEED); // Use defined bus speed and pins
  Wire.setTimeOut(ENDURANCE_I2C_TIMEOUT);
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);
  lastI2CActivity = millis();
  Serial.print("I2C slave initialized at ");
  Serial.print(ENDURANCE_I2C_BUS_SPEED);
  Serial.println(" Hz.");
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting Gateway...");
  printMacAddresses();
  Serial.println("Initializing ESP-NOW...");
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    while (true) {
      delay(1000);
    }
  }
  initializeEspNow();

  lastLedChangeTime = millis();

  // Initialize the gateway packet queue
  initializeGatewayQueue();
  initializeOutgoingQueue();

  delay(500); // Wait for horizon to start I2C master
  Serial.println("Initializing I2C as slave...");
  initI2CSlave();
}

void debugSerialToEspNow() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    Serial.print("Debug input: ");
    Serial.println(input);
    input.trim();
    if (input.length() > 0) {
      uint8_t *packet = (uint8_t *)malloc(input.length() + 1);
      if (packet) {
        memcpy(packet, input.c_str(), input.length());
        packet[input.length()] = '\0'; // Null-terminate the string
        enqueueOutgoingPacket(packet, input.length() + 1);
        Serial.print("Enqueued debug packet for probe via ESP-NOW, size: ");
        Serial.println(input.length() + 1);
      } else {
        Serial.println("Failed to allocate memory for debug packet.");
      }
    }
  }
}
#ifdef GATEWAY_DEBUG
void debugAddSampleDataToQueue() {
  // Create a sample FlightData packet
  FlightData sampleData;
  sampleData.timestamp = millis() - debugFlightStartTime; // Use elapsed time
  sampleData.pressure =
      1013.25 + sin(millis() / 1000.0) * 10; // Example pressure in hPa
  sampleData.temperature =
      25.0 + sin(millis() / 1000.0) * 5; // Example temperature in Celsius
  sampleData.altitude =
      0 + sin(millis() / 1000.0) * 100; // Example altitude in meters
  sampleData.ax = 1000;
  sampleData.ay = 2000;
  sampleData.az = 3000;
  sampleData.gx = 4000;
  sampleData.gy = 5000;
  sampleData.gz = 6000;
  sampleData.batteryLevel = 75; // Example battery level
  sampleData.isFlying = debugIsFlying;
  sampleData.isDeployed = debugIsDeployed;
  sampleData.isLanded = debugIsLanded;

  // Allocate memory for the packet
  uint8_t *packet = (uint8_t *)malloc(sizeof(FlightData));
  if (packet) {
    memcpy(packet, &sampleData, sizeof(FlightData));
    enqueueGatewayPacket(packet, sizeof(FlightData));
    // Serial.println("Sample data added to gateway queue.");
  } else {
    Serial.println("Failed to allocate memory for sample data packet.");
  }
}
#endif // GATEWAY_DEBUG

void loop() {
  static Timer sendTimer(DATA_SEND_INTERVAL); // Data send frequency
#ifdef GATEWAY_DEBUG
  static Timer debugSampleData(
      DATA_SEND_INTERVAL); // Debug sample data frequency
#endif

  debugSerialToEspNow();
  // Data send logic
  if (sendTimer.expired()) {
    // If there is a packet to send to the probe
    if (outgoingQueue.packetQueueSize > 0) {
      uint8_t *packet =
          outgoingQueue.packetQueue[outgoingQueue.currentPacketIndex];
      uint16_t packetSize =
          outgoingQueue.packetSizes[outgoingQueue.currentPacketIndex];
      uint8_t result = esp_now_send(probeMacAddress, packet, packetSize);
      if (result == 0) {
        Serial.println("Sent packet to probe via ESP-NOW.");
        dequeueOutgoingPacket();
      } else {
        Serial.print("Failed to send packet to probe, error: ");
        Serial.println(result);
      }
    }
  }
#ifdef GATEWAY_DEBUG
  if (debugSampleData.expired()) {
    debugAddSampleDataToQueue();
  }

  if (debugIsFlying && !debugIsDeployed &&
      millis() - debugFlightStartTime >= 2000) { // Simulate deployment
    debugIsDeployed = true;
    debugDeploymentTime = millis();
    Serial.println("Debug: Deployment simulated.");
  } else if (debugIsDeployed && !debugIsLanded &&
             millis() - debugDeploymentTime >= 5000) {
    debugIsLanded = true;
    debugIsFlying = false; // Simulate landing
    debugLandingTime = millis();
    Serial.println("Debug: Landing simulated.");
  } else if (debugIsLanded && millis() - debugLandingTime >= 5000) {
    debugIsFlying = false;
    debugIsDeployed = false;
    debugIsLanded = false; // Reset state
    Serial.println("Debug: Resetting state after landing.");
  }

#endif
  // I2C watchdog: re-init if no activity for 5 seconds
  if (millis() - lastI2CActivity > 5000) {
    Serial.println("I2C inactivity detected, re-initializing I2C slave...");
    initI2CSlave();
  }

  printDataRates();
}

void sendEspNowPacket(const uint8_t *mac_addr, const uint8_t *data,
                      size_t len) {
  esp_now_send(mac_addr, data, len);
  espNowBytesSentTotal += len;
}

void printDataRates() {
  static unsigned long lastPrint = 0;
  static unsigned long lastI2CBytesSent = 0;
  static unsigned long lastI2CBytesReceived = 0;
  static unsigned long lastEspNowBytesSent = 0;
  static unsigned long lastEspNowBytesReceived = 0;
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    unsigned long i2cRateSent = i2cBytesSentTotal - lastI2CBytesSent;
    unsigned long i2cRateReceived =
        i2cBytesReceivedTotal - lastI2CBytesReceived;
    unsigned long espNowRateSent = espNowBytesSentTotal - lastEspNowBytesSent;
    unsigned long espNowRateReceived =
        espNowBytesReceivedTotal - lastEspNowBytesReceived;
    Serial.print("[DataRate] I2C OUT: ");
    Serial.print(i2cRateSent);
    Serial.print(" B/s, I2C IN: ");
    Serial.print(i2cRateReceived);
    Serial.print(" B/s, ESP-NOW OUT: ");
    Serial.print(espNowRateSent);
    Serial.print(" B/s, ESP-NOW IN: ");
    Serial.print(espNowRateReceived);
    Serial.println(" B/s");
    lastI2CBytesSent = i2cBytesSentTotal;
    lastI2CBytesReceived = i2cBytesReceivedTotal;
    lastEspNowBytesSent = espNowBytesSentTotal;
    lastEspNowBytesReceived = espNowBytesReceivedTotal;
    lastPrint = now;
  }
}
