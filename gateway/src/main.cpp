#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <core.h>
#include <espnow.h>

#define PROBE_MAC_ADDRESS "4C:75:25:03:EA:DC"
#define I2C_SLAVE_ADDRESS 0x03

struct GatewayState {
  uint16_t packetQueueSize;
  uint16_t packetQueueMaxSize;
  uint16_t currentPacketIndex;
  uint8_t **packetQueue;
};

// The maximum duration (in seconds) for which the gateway will store data
// packets before data loss occurs.
int8_t dataQueueSizeSeconds = 10;

GatewayState gatewayState = {0, dataQueueSizeSeconds *dataSendFrequency, 0,
                             NULL};

bool ledState = false;
long unsigned lastLedChangeTime;

// --- Add this struct for the second queue ---
struct OutgoingQueue {
  uint8_t **packetQueue;
  uint16_t packetQueueSize;
  uint16_t packetQueueMaxSize;
  uint16_t currentPacketIndex;
};

#define OUTGOING_QUEUE_MAX_SIZE 16

OutgoingQueue outgoingQueue = {NULL, 0, OUTGOING_QUEUE_MAX_SIZE, 0};

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
  if (!outgoingQueue.packetQueue) {
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
    Serial.println("Gateway packet queue is full, discarding packet.");
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
    outgoingQueue.currentPacketIndex = (outgoingQueue.currentPacketIndex + 1) %
                                       outgoingQueue.packetQueueMaxSize;
    outgoingQueue.packetQueueSize--;
  }
}

void onDataReceived(unsigned char *mac_addr, unsigned char *data, u8 len) {
  FlightData flightData;
  Serial.print(", Data received: ");
  memcpy(&flightData, data, sizeof(FlightData));
  printFlightData(flightData);

  u8_t *packet = (u8_t *)malloc(len);
  if (packet) {
    memcpy(packet, data, len);
    enqueueGatewayPacket(packet, len);
  } else {
    Serial.println("Failed to allocate memory for received packet.");
  }

  if (millis() - lastLedChangeTime > ledBlinkDelay) {
    if (ledState) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    ledState = !ledState;
    lastLedChangeTime = millis();
  }
}

void onI2CRequest() {
  // Check if there is a packet in the queue
  if (gatewayState.packetQueueSize > 0) {
    uint8_t *packet = gatewayState.packetQueue[gatewayState.currentPacketIndex];
    size_t packetSize = sizeof(FlightData);

    Wire.write(packet, packetSize); // Send the packet to the master
    dequeueGatewayPacket();         // Remove it from the queue
  } else {
    // No packet available, send a single zero byte
    uint8_t empty = 0;
    Wire.write(&empty, 1);
  }
}

void onI2CReceive(int numBytes) {
  if (numBytes > 0) {
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
    } else {
      Serial.println("Failed to allocate memory for outgoing packet.");
      // Drain the buffer
      while (Wire.available())
        Wire.read();
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  printMacAddresses();
  Serial.println("Initializing ESP-NOW...");
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    while (true) {
      delay(1000);
    }
  }
  macAddressToByteArray(PROBE_MAC_ADDRESS, probeMacAddress);
  Serial.println("ESP-NOW initialized successfully.");
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  uint8_t probeMacAddress[6];
  macAddressToByteArray(PROBE_MAC_ADDRESS, probeMacAddress);
  esp_now_add_peer(probeMacAddress, ESP_NOW_ROLE_COMBO, 0, NULL, 0);
  esp_now_register_recv_cb(onDataReceived);

  lastLedChangeTime = millis();
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize the gateway packet queue
  initializeGatewayQueue();
  initializeOutgoingQueue(); // <-- Add this

  // Initialize I2C as slave
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);
}

void loop() {
  // If there is a packet to send to the probe
  if (outgoingQueue.packetQueueSize > 0) {
    uint8_t *packet =
        outgoingQueue.packetQueue[outgoingQueue.currentPacketIndex];

    // Send via ESP-NOW
    uint8_t result = esp_now_send(probeMacAddress, packet, sizeof(packet));
    if (result == 0) {
      Serial.println("Sent packet to probe via ESP-NOW.");
      dequeueOutgoingPacket();
    } else {
      Serial.print("Failed to send packet to probe, error: ");
      Serial.println(result);
    }
  }
}
