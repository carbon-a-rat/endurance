#include <Arduino.h>
#include <Timer.h>
#include <Wire.h>
#include <core.h>

#include "DFRobot_MPX5700SoftWire.h"

#define AIR_SENSOR_I2C_ADDRESS 0x16

#define CANCEL_PIN 13
#define AIR_DISTRIBUTOR_PIN 12
#define CYLINDER_PIN 11
#define WATER_VALVE_PIN 10

SoftWire Wire1(4, 5);

DFRobot_MPX5700 mpx5700(&Wire1, AIR_SENSOR_I2C_ADDRESS);

float targetAirPressure = 300.0; // in kPa
float targetWaterVolume = 0.5;   // in L

enum launchStates launchState = STAND_BY;

WaterLoadingData waterLoadingData;
AirLoadingData airLoadingData;

int sensorPin = 2;
volatile long pulse;

unsigned long launchTimestamp = 0; // Timestamp of the last launch
unsigned long cancelTimestamp = 0; // Timestamp of the last cancel

// Data rate counter for I2C requests
volatile unsigned long i2cRequestCount = 0;
unsigned long lastRatePrint = 0;
unsigned long lastRate = 0;

// Forward declarations
void increase();
void stopWaterFlow();
void stopAirFlow();
void fillWater();
void fillAir();
void cancelFlight();
void updateLoadingData();
void prepareDummyData();
void onI2CRequest();
void addWaterLoadingDataToQueue(const WaterLoadingData &data);
void addAirLoadingDataToQueue(const AirLoadingData &data);
void onI2CReceive(int numBytes);
void resetAfterLaunch();
void endCancelFlight();

const int BUFFER_SIZE = PAD_DATA_SEND_FREQUENCY * 4; // 4 seconds of data
struct CircularBuffer {
  PadDataPacket buffer[BUFFER_SIZE];
  int head = 0;
  int tail = 0;
  int count = 0;

  bool isFull() { return count == BUFFER_SIZE; }
  bool isEmpty() { return count == 0; }

  bool enqueue(const PadDataPacket &packet) {
    if (isFull()) {
      return false; // Buffer is full
    }
    buffer[head] = packet;
    head = (head + 1) % BUFFER_SIZE;
    count++;
    return true;
  }

  bool dequeue(PadDataPacket &packet) {
    if (isEmpty()) {
      return false; // Buffer is empty
    }
    packet = buffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    count--;
    return true;
  }
};

CircularBuffer padDataQueue;

void updateLoadingData() {
  long currentPulse = pulse;
  waterLoadingData.waterVolume = currentPulse / 660.0;
  waterLoadingData.error = waterLoadingData.waterVolume - targetWaterVolume;
  waterLoadingData.waterFlowRate = waterLoadingData.waterVolume /
                                   (millis() - waterLoadingData.timestamp) *
                                   1000.0;
  waterLoadingData.timestamp = millis();

  airLoadingData.pressure = mpx5700.getPressureValue_kpa(1);
  airLoadingData.error = airLoadingData.pressure - targetAirPressure;
  airLoadingData.timestamp = millis();

  Serial.println(airLoadingData.pressure);
}

void onI2CRequest() {
  if (!padDataQueue.isEmpty()) {
    PadDataPacket packet;
    if (padDataQueue.dequeue(packet)) {
      Wire.write((uint8_t *)&packet, sizeof(PadDataPacket));
    }
  } else {
    // Send an empty packet if the queue is empty
    PadDataPacket emptyPacket;
    emptyPacket.type = PadDataPacket::NO_DATA;
    emptyPacket.launchState = launchState;
    Wire.write((uint8_t *)&emptyPacket, sizeof(PadDataPacket));
  }
}

void addWaterLoadingDataToQueue(const WaterLoadingData &data) {
  PadDataPacket packet;
  packet.launchState = launchState;
  packet.type = PadDataPacket::WATER_LOADING_DATA;
  packet.data.waterLoadingData = data;
  if (!padDataQueue.enqueue(packet)) {
    Serial.println("Queue is full. Dropping water loading data.");
  }
}

void addAirLoadingDataToQueue(const AirLoadingData &data) {
  PadDataPacket packet;
  packet.launchState = launchState;
  packet.type = PadDataPacket::AIR_LOADING_DATA;
  packet.data.airLoadingData = data;
  if (!padDataQueue.enqueue(packet)) {
    Serial.println("Queue is full. Dropping air loading data.");
  }
}

void sendLoadingData() {
  if (launchState == FILLING_WATER || launchState == FILLED_WATER) {
    // Add water loading data to the queue
    addWaterLoadingDataToQueue(waterLoadingData);
  } else if (launchState == FILLING_AIR || launchState == READY_TO_LAUNCH) {
    // Add air loading data to the queue
    addAirLoadingDataToQueue(airLoadingData);
  }
}

void initializePins() {
  pinMode(AIR_DISTRIBUTOR_PIN, OUTPUT);
  pinMode(CYLINDER_PIN, OUTPUT);
  pinMode(CANCEL_PIN, OUTPUT);
  pinMode(WATER_VALVE_PIN, OUTPUT);

  pinMode(sensorPin, INPUT);
}

void initializeI2C() {
  Wire.begin(PAD_I2C_ADDRESS);
  Wire.setClock(ENDURANCE_I2C_BUS_SPEED);
  Wire.setWireTimeout(ENDURANCE_I2C_TIMEOUT);
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);
  Serial.print("I2C Slave ready at address 0x");
  Serial.println(PAD_I2C_ADDRESS, HEX);
}

void initializeMPX5700() {
  while (!mpx5700.begin()) {
    Serial.println("MPX5700 begin fail, please check connect!");
    delay(1000);
  }
  Serial.println("i2c begin success");
  mpx5700.setMeanSampleSize(5);
}

void setup() {
  Serial.begin(115200);

  initializePins();
  attachInterrupt(digitalPinToInterrupt(sensorPin), increase, RISING);

  initializeI2C();
  initializeMPX5700();

  Serial.println("Setup done. Filling air in 5 sec");
  delay(5000);
  fillAir();
}

void loop() {

  static Timer dataRateTimer(1000);
  static Timer sensorCheckTimer(
      DATA_SEND_INTERVAL); // Timer for checking sensors
  static Timer dataSendTimer(PAD_DATA_SEND_INTERVAL); // Timer for sending data

  // Print I2C data rate every second
  if (dataRateTimer.expired()) {
    lastRate = i2cRequestCount;
    i2cRequestCount = 0;
    lastRatePrint = millis();
    unsigned long bytesPerSecond = lastRate * PAD_I2C_CHUNK_SIZE;
    Serial.print("I2C requests per second: ");
    Serial.print(lastRate);
    Serial.print(", Data rate: ");
    Serial.print(bytesPerSecond);
    Serial.println(" B/s");
  }

  if (sensorCheckTimer.expired()) {
    updateLoadingData();
    if (launchState == FILLING_AIR) {
      // Checks pressure sensor

      float currentAirPressure = airLoadingData.pressure;
      if (currentAirPressure >= targetAirPressure) {
        // Target air pressure reached. Now closing valve.
        launchState = READY_TO_LAUNCH;
        stopAirFlow();
      }
    }

    if (launchState == FILLING_WATER) {
      // Checks water flow sensor

      float currentWaterVolume = waterLoadingData.waterVolume;

      if (currentWaterVolume >= targetWaterVolume) {
        // Target water volume. Now closing valve.
        launchState = FILLED_WATER;
        stopWaterFlow();
      }
    }
  }

  if (launchState == LAUNCHING && millis() - launchTimestamp >= 1000) {
    // Wait for 1 second after launch to reset
    resetAfterLaunch();
  }

  if (launchState == CANCELLING && millis() - cancelTimestamp >= 1000) {
    // Wait for 1 second after cancel to end the procedure
    endCancelFlight();
  }

  if (dataSendTimer.expired()) {
    sendLoadingData();
  }
}

void launchFlight() {
  if (launchState == READY_TO_LAUNCH) {
    Serial.println("Launching.");
    // Launch the rocket
    digitalWrite(CYLINDER_PIN, HIGH);
    launchState = LAUNCHING;
    launchTimestamp = millis(); // Record the launch time
  } else {
    Serial.println("WARNING: Launch not ready yet.");
  }
}

void resetAfterLaunch() {
  if (launchState == LAUNCHING) {
    digitalWrite(CYLINDER_PIN, LOW);

    launchState = STAND_BY;
  }
}

void cancelFlight() {
  Serial.println("Cancelling flight...");
  launchState = CANCELLING;
  cancelTimestamp = millis(); // Record the cancel time

  stopWaterFlow();
  stopAirFlow();

  // Cancels the flight by releasing the air / water contained in the rocket
  digitalWrite(CANCEL_PIN, HIGH);
}

void endCancelFlight() {
  // Ends the cancel flight procedure.
  digitalWrite(CANCEL_PIN, LOW);
  Serial.println("Flight cancelled.");
  launchState = CANCELLED;
}

void fillAir() {
  // Starts to fill the rocket with air.

  // Doesn't close the cancel_pin as it should already be closed.
  if (launchState != FILLED_WATER) {
    Serial.println("WARNING: Water hasn't been filled yet !");
    return;
  }

  // Closes the circuit.
  digitalWrite(CANCEL_PIN, LOW);

  digitalWrite(AIR_DISTRIBUTOR_PIN, HIGH);
  Serial.println("Starting to fill air...");

  launchState = FILLING_AIR;
}

void fillWater() {
  // Starts to fill the rocket with water.

  if (launchState == FILLED_WATER) {
    Serial.println("WARNING: Water has already been filled !");
  }

  // Resets the pulse to start counting again.
  pulse = 0;

  // Closes the circuit.
  digitalWrite(CANCEL_PIN, LOW);

  digitalWrite(WATER_VALVE_PIN, HIGH);
  Serial.println("Starting to fill water...");

  launchState = FILLING_WATER;
}

void stopWaterFlow() {
  // Stops the water flow
  digitalWrite(WATER_VALVE_PIN, LOW);

  Serial.println("Water valve closed");
}

void stopAirFlow() {
  // Stops the air flow
  digitalWrite(AIR_DISTRIBUTOR_PIN, LOW);

  Serial.println("Air valve closed");
}

void increase() { pulse++; }

void onI2CReceive(int numBytes) {
  if (numBytes < 1) {
    Serial.println("Received empty data");
    return;
  }
  char tempBuf[64];
  size_t copyLen =
      (numBytes < (int)(sizeof(tempBuf) - 1)) ? numBytes : (sizeof(tempBuf) - 1);
  Wire.readBytes(tempBuf, copyLen);
  tempBuf[copyLen] = '\0';
  String command(tempBuf);
  command.trim();

  if (command == "launch") {
    if (launchState == READY_TO_LAUNCH) {
      launchFlight();
    } else {
      Serial.println("Cannot launch: not ready.");
    }
  } else if (command == "cancel") {
    if (launchState != STAND_BY) {
      cancelFlight();
    } else {
      Serial.println("Cannot cancel: no flight in progress.");
    }
  } else if (command.startsWith("fill_air")) {
    if (launchState == FILLED_WATER) {
      targetAirPressure = command.substring(9).toFloat();
      fillAir();
    } else {
      Serial.println("Cannot fill air: water not filled yet.");
    }
  } else if (command.startsWith("fill_water")) {
    if (launchState == STAND_BY) {
      targetWaterVolume = command.substring(11).toFloat();
      fillWater();
    } else {
      Serial.println("Cannot fill water: already in progress.");
    }
  } else if (command == "send_data") {
    // This command is used to send the current loading data
    addAirLoadingDataToQueue(airLoadingData);
    addWaterLoadingDataToQueue(waterLoadingData);
    Serial.println("Loading data sent.");
  } else {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}