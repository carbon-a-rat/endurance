#include "main.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPBattery.h>
#include <Sensors.h>
#include <Servo.h>
#include <Timer.h>
#include <espnow.h>

// Core Endurance library
#include <EnduranceConfig.h>
#include <core.h>

struct ProbeState {
  bool isDeployed;
  bool isFlying;
  bool isLanded;
  float batteryLevel;
  uint8_t gatewayAddress[6];
  uint8_t **packetQueue;
  uint16_t currentPacketIndex;
  uint16_t packetQueueSize;
  uint16_t packetQueueMaxSize;
  bool sendingPacket;
  long unsigned launchTime = 0;
  long unsigned deploymentTime = 0;
  long unsigned landingTime = 0;
  float maxAltitude = 0.0f;    // Track max altitude for apogee detection
  bool apogeeDetected = false; // Flag for apogee detection
  float lastAltitude = 0.0f;
  unsigned long altitudeStableStart = 0;
};

float baseAltitude;
Servo servo;
int servoPin = 14;

ESPBattery battery;
int batteryPin = A0;

BarometerSensor sensor(0x77);
AccGyroSensor accGyroSensor(0x6A);

long expectedFlightTime = 20;

bool shouldPrintFlightData = false;
bool shouldPrintBatteryData = true;
bool shouldPrintNetworkData = false;

ProbeState probeState = {
    false, false, false,
    0,     {0},   NULL,
    0,     0,     (uint16_t)(expectedFlightTime *DATA_SEND_FREQUENCY),
    false, 0,     0};

long unsigned t0 = 0;
long unsigned lastSendTime = 0;

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

void batteryStateHandler(ESPBattery &b) {

  int batteryPercentage = b.getPercentage();
  probeState.batteryLevel = batteryPercentage;

  if (shouldPrintBatteryData) {
    Serial.print("Battery Level: ");
    Serial.print(batteryPercentage);
    Serial.print("%, Level: ");
    Serial.print(b.getLevel());
    Serial.print("mV, State: ");
    Serial.println(b.stateToString(b.getState()));
  }
}

void initializeSensors() {
  Serial.println("Initializing sensors...");
  if (sensor.initialize() == false) {
    Serial.println("Failed to initialize sensor!");
  } else {
    Serial.println("Barometer sensor initialized successfully.");
    Serial.print("I2C Address:");
    Serial.println(sensor.getI2CAddress(), HEX);
    Serial.println("Reading sensor data...");
  }

  if (accGyroSensor.initialize() == false) {
    Serial.println("Failed to initialize accelerometer/gyroscope sensor!");
  } else {
    Serial.println("Accelerometer/Gyroscope sensor initialized successfully.");
    Serial.print("I2C Address:");
    Serial.println(accGyroSensor.getI2CAddress(), HEX);
    Serial.println("Reading sensor data...");
  }
  delay(500); // Wait for the sensors to stabilize
  Serial.println("Taking baseline altitude reading...");
  // Take a baseline altitude reading
  float tempPressure, tempTemperature, tempAltitude;
  sensor.read(tempPressure, tempTemperature, tempAltitude);
  baseAltitude = tempAltitude;
  Serial.print("Base altitude set to: ");
  Serial.println(baseAltitude);
}

void printMacAddresses() {
  Serial.print("Probe MAC Address:  ");
  Serial.println(WiFi.macAddress());
  Serial.print("Gateway MAC Address:  ");
  Serial.println(GATEWAY_MAC_ADDRESS);
}

void sendDataToQueue(FlightData &flightData) {
  // Zero-initialize the packet for safety
  uint8_t *packet = (uint8_t *)calloc(1, sizeof(FlightData));
  if (!packet) {
    if (shouldPrintNetworkData) {
      Serial.println("Failed to allocate memory for packet.");
    }
    return;
  }
  memcpy(packet, &flightData, sizeof(FlightData));

  // Add the packet to the queue
  uint16_t newPacketIndex =
      (probeState.currentPacketIndex + probeState.packetQueueSize) %
      probeState.packetQueueMaxSize;
  if (probeState.packetQueueSize < probeState.packetQueueMaxSize) {
    probeState.packetQueue[newPacketIndex] = packet;
    probeState.packetQueueSize++;
  } else {
    if (shouldPrintNetworkData) {
      Serial.println("Packet queue is full, discarding packet.");
    }
    free(packet);
  }
}

void sendDataToGateway() {
  if (probeState.packetQueueSize != 0 && !probeState.sendingPacket) {
    esp_now_send(probeState.gatewayAddress,
                 probeState.packetQueue[probeState.currentPacketIndex],
                 sizeof(FlightData));
    probeState.sendingPacket = true;
  }
}

void onDataSent(unsigned char *mac_addr, u8 status) {
  if (status == 0) {
    if (shouldPrintNetworkData) {
      Serial.println("Data sent successfully");
    }

    // Free the sent packet
    free(probeState.packetQueue[probeState.currentPacketIndex]);
    probeState.currentPacketIndex =
        (probeState.currentPacketIndex + 1) % probeState.packetQueueMaxSize;
    probeState.packetQueueSize--;
  } else {
    if (shouldPrintNetworkData) {
      Serial.print("Error sending data, status: ");
      Serial.println(status);
    }
  }
  probeState.sendingPacket = false;
}

void startFlight() {
  probeState.isFlying = true;
  probeState.launchTime = millis();
  float tempTemp, tempPressure;
  sensor.read(tempPressure, tempTemp, baseAltitude); // Zero the altitude
}

void onDataReceived(unsigned char *mac_addr, unsigned char *data, u8 len) {
  String cmd = String((char *)data).substring(0, len);

  if (cmd.startsWith("GO")) {
    Serial.println("Received GO command");
    startFlight();
  } else if (cmd.startsWith("RESET")) {
    Serial.println("Received RESET command");
    resetFlight();
  } else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

void rotateAccGyroData(float &ax, float &ay, float &az, float &gx, float &gy,
                       float &gz) {
  // Rotate the accelerometer and gyroscope data according to the sensor's
  // orientation : upside down (180 deg about X axis)
  float tempAx = ax;
  float tempAy = ay;
  float tempAz = az;
  float tempGx = gx;
  float tempGy = gy;
  float tempGz = gz;

  ax = tempAx;
  ay = -tempAy;
  az = -tempAz;
  gx = tempGx;
  gy = -tempGy;
  gz = -tempGz;
}

void deployementCheck(FlightData &flightData) {
  if (probeState.isFlying && !probeState.isDeployed &&
      flightData.timestamp > PROBE_DEPLOYMENT_TIME_THRESHOLD) {

    // Track max altitude
    if (flightData.altitude > probeState.maxAltitude) {
      probeState.maxAltitude = flightData.altitude;
      probeState.apogeeDetected = false;
    }

    // Detect apogee: altitude has dropped by threshold after max
    if (!probeState.apogeeDetected &&
        (probeState.maxAltitude - flightData.altitude) >
            PROBE_DEPLOYMENT_ALTITUDE_DROP_AFTER_APOGEE) {
      probeState.apogeeDetected = true;
      probeState.isDeployed = true;
      probeState.deploymentTime = millis();
      servo.write(PROBE_SERVO_DEPLOYED_POSITION); // Deploy the parachute
      Serial.println("Parachute deployed at apogee!");
    }
  }
}

void landingCheck(FlightData &flightData) {
  if (probeState.isFlying && probeState.isDeployed) {
    float altitudeChange = fabs(flightData.altitude - probeState.lastAltitude);

    if (altitudeChange < PROBE_LANDING_ALTITUDE_STABLE_WINDOW) {
      if (probeState.altitudeStableStart == 0) {
        probeState.altitudeStableStart = millis();
      } else if (millis() - probeState.altitudeStableStart >
                 PROBE_LANDING_ALTITUDE_STABLE_TIME) {
        probeState.isLanded = true;
        probeState.isFlying = false;
        probeState.landingTime = millis();
        Serial.println("Landed (altitude stable)!");
      }
    } else {
      probeState.altitudeStableStart = 0;
    }
    probeState.lastAltitude = flightData.altitude;
  }
}

void resetFlightCheck() {
  if (probeState.isLanded && millis() - probeState.landingTime > 1000) {
    resetFlight();
    startFlight(); // TESTING: Restart instantly after landing
  }
}

void resetFlight() {
  probeState.isDeployed = false;
  probeState.isFlying = false;
  probeState.isLanded = false;
  probeState.launchTime = 0;
  probeState.deploymentTime = 0;
  probeState.landingTime = 0;
  servo.write(PROBE_SERVO_INITIAL_POSITION); // Reset the servo position
  probeState.maxAltitude = 0.0f;
  probeState.apogeeDetected = false;
  probeState.lastAltitude = 0.0f;
  probeState.altitudeStableStart = 0;
}

FlightData getFlightData() {
  FlightData flightData;
  flightData.timestamp = millis() - probeState.launchTime;
  sensor.read(flightData.pressure, flightData.temperature, flightData.altitude);
  flightData.altitude -= baseAltitude;
  accGyroSensor.read(flightData.ax, flightData.ay, flightData.az, flightData.gx,
                     flightData.gy, flightData.gz);

  flightData.batteryLevel = probeState.batteryLevel;

  rotateAccGyroData(flightData.ax, flightData.ay, flightData.az, flightData.gx,
                    flightData.gy, flightData.gz);

  deployementCheck(flightData);
  landingCheck(flightData);
  resetFlightCheck();
  flightData.isFlying = probeState.isFlying;
  flightData.isDeployed = probeState.isDeployed;
  flightData.isLanded = probeState.isLanded;
  return flightData;
}

void initializeESPNOW() {
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
  const char *gatewayMacAddress = GATEWAY_MAC_ADDRESS;
  macAddressToByteArray(gatewayMacAddress, probeState.gatewayAddress);
  esp_now_add_peer(probeState.gatewayAddress, ESP_NOW_ROLE_COMBO, 0, NULL, 0);
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
}

void setup() {
  pinMode(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  Serial.println();

  printMacAddresses();

  initializeSensors();

  initializeESPNOW();

  battery.begin(batteryPin);
  battery.setLevelChangedHandler(batteryStateHandler);
  batteryStateHandler(battery);

  servo.attach(servoPin);

  probeState.packetQueue =
      (uint8_t **)calloc(probeState.packetQueueMaxSize, sizeof(uint8_t *));
  if (!probeState.packetQueue) {
    Serial.println("Failed to allocate packet queue!");
    while (true)
      delay(1000);
  }
  Serial.print("Free heap after queue alloc: ");
  Serial.println(ESP.getFreeHeap());

  t0 = millis();
  lastSendTime = t0;
  servo.write(PROBE_SERVO_INITIAL_POSITION); // Initial position of the servo
  Serial.println("Setup complete.");
}
void loop() {
  static Timer sendTimer(DATA_SEND_INTERVAL);

  if (sendTimer.expired()) {
    FlightData flightData = getFlightData();
    sendDataToQueue(flightData);
    if (shouldPrintFlightData) {
      printFlightData(flightData);
    }
  }
  battery.loop();
  sendDataToGateway();
}
