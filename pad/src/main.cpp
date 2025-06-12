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

enum launchingStates { STAND_BY, FILLING_AIR, FILLING_WATER, READY_TO_LAUNCH };

enum launchingStates launchingState = STAND_BY;

WaterLoadingData waterLoadingData;
AirLoadingData airLoadingData;

int sensorPin = 2;
volatile long pulse;

bool waterFilled;
bool airFilled;

// Data rate counter for I2C requests
volatile unsigned long i2cRequestCount = 0;
unsigned long lastRatePrint = 0;
unsigned long lastRate = 0;

Timer dataUpdateTimer(DATA_SEND_INTERVAL);
Timer sensorCheckTimer(DATA_SEND_INTERVAL);

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

const int BUFFER_SIZE = DATA_SEND_FREQUENCY * 10; // 10 seconds of data

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
  waterLoadingData.timestamp = millis();

  airLoadingData.pressure = mpx5700.getPressureValue_kpa(1);
  airLoadingData.timestamp = millis();

  Serial.println(airLoadingData.pressure);

  addAirLoadingDataToQueue(airLoadingData);
  addWaterLoadingDataToQueue(waterLoadingData);
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
    Wire.write((uint8_t *)&emptyPacket, sizeof(PadDataPacket));
  }
}

void addWaterLoadingDataToQueue(const WaterLoadingData &data) {
  PadDataPacket packet;
  packet.type = PadDataPacket::WATER_LOADING_DATA;
  packet.data.waterLoadingData = data;
  if (!padDataQueue.enqueue(packet)) {
    Serial.println("Queue is full. Dropping water loading data.");
  }
}

void addAirLoadingDataToQueue(const AirLoadingData &data) {
  PadDataPacket packet;
  packet.type = PadDataPacket::AIR_LOADING_DATA;
  packet.data.airLoadingData = data;
  if (!padDataQueue.enqueue(packet)) {
    Serial.println("Queue is full. Dropping air loading data.");
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
  // Update loading data every 200 ms

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
    if (launchingState == FILLING_AIR) {
      // Checks pressure sensor

      float currentAirPressure = mpx5700.getPressureValue_kpa(1);
      if (currentAirPressure >= targetAirPressure) {
        // Target air pressure reached. Now closing valve.

        stopAirFlow();
      }
    }

    if (launchingState == FILLING_WATER) {
      // Checks water flow sensor

      long currentPulse = pulse;
      float currentWaterVolume = currentPulse / 660.0;

      if (currentWaterVolume >= targetWaterVolume) {
        // Target water volume. Now closing valve.

        stopWaterFlow();
      }
    }
  }
}

void launchFlight() {
  if (launchingState == READY_TO_LAUNCH) {
    Serial.println("Launching.");
    // Launch the rocket
    digitalWrite(CYLINDER_PIN, HIGH);
    delay(2000);
    digitalWrite(CYLINDER_PIN, LOW);

    waterFilled = false;
    airFilled = false;
    launchingState = STAND_BY;
  } else {
    Serial.println("WARNING: Launch not ready yet.");
  }
}

void cancelFlight() {
  Serial.println("Cancelling flight...");

  stopWaterFlow();
  stopAirFlow();

  // Cancels the flight by releasing the air / water contained in the rocket
  digitalWrite(CANCEL_PIN, HIGH);

  Serial.println("Flight cancelled.");
}

void fillAir() {
  // Starts to fill the rocket with air.

  // Doesn't close the cancel_pin as it should already be closed.
  if (airFilled == true) {
    Serial.println("WARNING: Air has already been filled !");
    return;
  }
  /*if (waterFilled == false) {
    Serial.println("WARNING: Water hasn't been filled yet !");
    return;
  }*/

  // Closes the circuit.
  digitalWrite(CANCEL_PIN, LOW);

  digitalWrite(AIR_DISTRIBUTOR_PIN, HIGH);
  Serial.println("Starting to fill air...");

  launchingState = FILLING_AIR;

  airLoadingData.isLoading = true;
}

void fillWater() {
  // Starts to fill the rocket with water.

  if (waterFilled == true) {
    Serial.println("WARNING: Water has already been filled !");
  }

  // Resets the pulse to start counting again.
  pulse = 0;

  // Closes the circuit.
  digitalWrite(CANCEL_PIN, LOW);

  digitalWrite(WATER_VALVE_PIN, HIGH);
  Serial.println("Starting to fill water...");

  launchingState = FILLING_WATER;

  waterLoadingData.isLoading = true;
}

void stopWaterFlow() {
  // Stops the water flow
  digitalWrite(WATER_VALVE_PIN, LOW);

  Serial.println("Water valve closed");

  launchingState = STAND_BY;

  waterLoadingData.isLoading = false;
}

void stopAirFlow() {
  // Stops the air flow
  digitalWrite(AIR_DISTRIBUTOR_PIN, LOW);

  Serial.println("Air valve closed");

  launchingState = STAND_BY;

  airLoadingData.isLoading = false;
}

void increase() { pulse++; }
