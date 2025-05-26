#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPBattery.h>
#include <Sensors.h>
#include <Servo.h>
#include <Timer.h>
#include <core.h>
#include <espnow.h>

#define GATEWAY_MAC_ADDRESS "bc:ff:4d:40:d3:02"
#define DEPLOYMENT_ACC_THRESHOLD 1.5
#define DEPLOYMENT_TIME_THRESHOLD 300

#define LANDING_ACC_MIN_THRESHOLD 0.8
#define LANDING_ACC_MAX_THRESHOLD 1.2
#define LANDING_TIME_THRESHOLD 1000

#define SERVO_INITIAL_POSITION 0
#define SERVO_DEPLOYED_POSITION 180

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
bool shouldPrintBatteryData = false;
bool shouldPrintNetworkData = false;

ProbeState probeState = {
    false, false, false,
    0,     {0},   NULL,
    0,     0,     (uint16_t)(expectedFlightTime *dataSendFrequency),
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
    Serial.print("%, Voltage: ");
    Serial.print(b.getVoltage());
    Serial.print("V, State: ");
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
  // Allocate memory for the packet
  uint8_t *packet = (uint8_t *)malloc(sizeof(FlightData));
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

void onDataReceived(unsigned char *mac_addr, unsigned char *data, u8 len) {
  String cmd = String((char *)data).substring(0, len);

  if (cmd.startsWith("GO")) {
    Serial.println("Received GO command");
    probeState.isFlying = true;
    probeState.launchTime = millis();
    float tempTemp, tempPressure;
    sensor.read(tempPressure, tempTemp, baseAltitude); // Zero the altitude
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
      flightData.timestamp > DEPLOYMENT_TIME_THRESHOLD) {
    uint8_t accNorm =
        sqrt(flightData.ax * flightData.ax + flightData.ay * flightData.ay +
             flightData.az * flightData.az);
    if (accNorm < DEPLOYMENT_ACC_THRESHOLD) {
      probeState.isDeployed = true;
      probeState.deploymentTime = millis();
      servo.write(SERVO_DEPLOYED_POSITION); // Deploy the parachute
      Serial.println("Parachute deployed!");
    }
  }
}

void landingCheck(FlightData &flightData) {
  if (probeState.isFlying && probeState.isDeployed &&
      millis() - probeState.deploymentTime > LANDING_TIME_THRESHOLD) {
    uint8_t accNorm =
        sqrt(flightData.ax * flightData.ax + flightData.ay * flightData.ay +
             flightData.az * flightData.az);
    if (accNorm > LANDING_ACC_MIN_THRESHOLD &&
        accNorm < LANDING_ACC_MAX_THRESHOLD) {
      probeState.isLanded = true;
      probeState.isFlying = false;
      probeState.landingTime = millis();
      Serial.println("Landed!");
    }
  }
}

void resetFlightCheck() {
  if (probeState.isLanded && millis() - probeState.landingTime > 1000) {
    resetFlight();
  }
}

void resetFlight() {
  probeState.isDeployed = false;
  probeState.isFlying = false;
  probeState.isLanded = false;
  probeState.launchTime = 0;
  probeState.deploymentTime = 0;
  probeState.landingTime = 0;
  servo.write(SERVO_INITIAL_POSITION); // Reset the servo position
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
  servo.write(SERVO_INITIAL_POSITION); // Initial position of the servo
  Serial.println("Setup complete.");
}

void loop() {
  static Timer sendTimer(dataSendDelay);

  if (sendTimer.expired()) {
    FlightData flightData = getFlightData();
    sendDataToQueue(flightData);
    if (shouldPrintFlightData) {
      printFlightData(flightData);
    }
  }
  battery.loop();
  sendDataToGateway();
  // servo.write(0-180) for testing;
}
