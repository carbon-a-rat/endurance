#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define PROBE_MAC_ADDRESS "4C:75:25:03:EA:DC"

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
}

void loop() {}
