#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <core.h>
#include <espnow.h>

#define PROBE_MAC_ADDRESS "4C:75:25:03:EA:DC"

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

void printMacAddresses() {
  Serial.print("Probe MAC Address:  ");
  Serial.println(PROBE_MAC_ADDRESS);
  Serial.print("Gateway MAC Address:  ");
  Serial.println(WiFi.macAddress());
}

void onDataReceived(unsigned char *mac_addr, unsigned char *data, u8 len) {
  Serial.print(", Data received: ");
  for (int i = 0; i < len; i++) {
    Serial.print((char)data[i]);
  }
  Serial.println();

  if (millis() - lastLedChangeTime > 0) {
    if (ledState) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    ledState = !ledState;
    lastLedChangeTime = millis();
  }
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

// Add a packet to the queue
void enqueueGatewayPacket(uint8_t *packet, size_t packetSize) {
  uint16_t newPacketIndex =
      (gatewayState.currentPacketIndex + gatewayState.packetQueueSize) %
      gatewayState.packetQueueMaxSize;
  if (gatewayState.packetQueueSize < gatewayState.packetQueueMaxSize) {
    gatewayState.packetQueue[newPacketIndex] = (uint8_t *)malloc(packetSize);
    if (gatewayState.packetQueue[newPacketIndex]) {
      memcpy(gatewayState.packetQueue[newPacketIndex], packet, packetSize);
      gatewayState.packetQueueSize++;
    } else {
      Serial.println("Failed to allocate memory for gateway packet.");
    }
  } else {
    Serial.println("Gateway packet queue is full, discarding packet.");
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
}

void loop() {}
