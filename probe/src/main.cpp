#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPBattery.h>
#include <Sensors.h>
#include <Servo.h>
#include <espnow.h>

#define GATEWAY_MAC_ADDRESS "44:17:93:0F:00:BF"
#define DEPLOYMENT_ACC_THRESHOLD 1.5
#define DEPLOYMENT_TIME_THRESHOLD 300

#define LANDING_ACC_MIN_THRESHOLD 0.8
#define LANDING_ACC_MAX_THRESHOLD 1.2
#define LANDING_TIME_THRESHOLD 1000

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

struct SensorZeroValues {
  float altitude;
  float ax, ay, az;
};

Servo servo;
int servoPin = 14;

ESPBattery battery;
int batteryPin = A0;

BarometerSensor sensor(0x77);
AccGyroSensor accGyroSensor(0x6A);

int dataSendFrequency = 32; // 32 Hz
long dataSendDelay = 1000 / dataSendFrequency;

long expectedFlightTime = 20;

bool shouldPrintFlightData = false;
ProbeState probeState = {false, false, false,
                         0,     {0},   NULL,
                         0,     0,     expectedFlightTime *dataSendFrequency,
                         false, 0,     0};

SensorZeroValues sensorZeroValues = {0, 0, 0, 0};

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
  Serial.print("Battery Level: ");
  Serial.print(batteryPercentage);
  Serial.print("%, Voltage: ");
  Serial.print(b.getVoltage());
  Serial.print("V, State: ");
  Serial.println(b.stateToString(b.getState()));
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
    Serial.println("Failed to allocate memory for packet.");
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
    Serial.println("Packet queue is full, discarding packet.");
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
    Serial.println("Data sent successfully");
    // Free the sent packet
    free(probeState.packetQueue[probeState.currentPacketIndex]);
    probeState.currentPacketIndex =
        (probeState.currentPacketIndex + 1) % probeState.packetQueueMaxSize;
    probeState.packetQueueSize--;
  } else {
    Serial.println("Error sending data");
  }
  probeState.sendingPacket = false;
}

void onDataReceived(unsigned char *mac_addr, unsigned char *data, u8 len) {
  if (data[0] == 'G' && data[1] == 'O') {
    Serial.println("Received GO command");
    probeState.isFlying = true;
    probeState.launchTime = millis();
  }
}

void rotateAccGyroData(float &ax, float &ay, float &az, float &gx, float &gy,
                       float &gz) {
  // Rotate the accelerometer and gyroscope data according to the sensor's
  // orientation
  float tempAx = ax;
  float tempAy = ay;
  float tempAz = az;
  float tempGx = gx;
  float tempGy = gy;
  float tempGz = gz;
  // TODO: Implement the rotation logic based on the sensor's orientation

  // For now, just copy the values
  ax = tempAx;
  ay = tempAy;
  az = tempAz;
  gx = tempGx;
  gy = tempGy;
  gz = tempGz;
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
      servo.write(180); // Deploy the parachute
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
      servo.write(0); // Bring the servo back to the initial position
      Serial.println("Landed!");
    }
  }
}

void resetFlightCheck() {
  if (probeState.isLanded && millis() - probeState.landingTime > 1000) {
    probeState.isDeployed = false;
    probeState.isFlying = false;
    probeState.isLanded = false;
    probeState.launchTime = 0;
    probeState.deploymentTime = 0;
    probeState.landingTime = 0;
  }
}

FlightData getFlightData() {
  FlightData flightData;
  flightData.timestamp = millis() - probeState.launchTime;
  sensor.read(flightData.pressure, flightData.temperature, flightData.altitude);
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
  servo.write(0); // Initial position of the servo
  Serial.println("Setup complete.");
}

void printData(float p, float t, float a, float ax, float ay, float az,
               float gx, float gy, float gz) {
  Serial.print("Pressure: ");
  Serial.print(p);
  Serial.print(" hPa, Temperature: ");
  Serial.print(t);
  Serial.print(" °C, Altitude: ");
  Serial.print(a);
  Serial.print(" m, Accelerometer: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.print(az);
  Serial.print(" Gyroscope: ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.print(gz);
  Serial.println(" dps");
}

void loop() {
  if (millis() - lastSendTime >= dataSendDelay) {
    lastSendTime = millis();

    FlightData flightData = getFlightData();

    sendDataToQueue(flightData);

    if (shouldPrintFlightData) {
      printData(flightData.pressure, flightData.temperature,
                flightData.altitude, flightData.ax, flightData.ay,
                flightData.az, flightData.gx, flightData.gy, flightData.gz);
    }
  }
  battery.loop();
  sendDataToGateway();
  // servo.write(0-180);
}
