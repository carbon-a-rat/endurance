#include <Arduino.h>
#include <Wire.h>
#include <core.h>
#include <Timer.h>

#include "DFRobot_MPX5700.h"
#define I2C_ADDRESS 0x16

#define CANCEL_PIN 13
#define AIR_DISTRIBUTOR_PIN 12
#define CYLINDER_PIN 11
#define WATER_VALVE_PIN 10

DFRobot_MPX5700 mpx5700(&Wire, I2C_ADDRESS);

const float targetAirPressure = 200.0; // in kPa
float targetWaterVolume = 0.5;         // in L

enum launchingStates { STAND_BY, FILLING_AIR, FILLING_WATER, READY_TO_LAUNCH };

enum launchingStates launching_state = STAND_BY;

WaterLoadingData waterLoadingData;
AirLoadingData airLoadingData;

int sensorPin = 2;
volatile long pulse;

bool waterFilled;
bool airFilled;

// Dummy data buffer for I2C response
uint8_t i2cDummyData[PAD_I2C_CHUNK_SIZE] = {0};
unsigned long lastDummyUpdate = 0;

// Data rate counter for I2C requests
volatile unsigned long i2cRequestCount = 0;
unsigned long lastRatePrint = 0;
unsigned long lastRate = 0;

Timer timer(200);

// Forward declarations
void increase();
void stopWaterFlow();
void stopAirFlow();
void fillWater();
void fillAir();
void updateLoadingData();
void prepareDummyData();
void onI2CRequest();

void prepareDummyData() {
  // Fill dummy data (for now, just incrementing bytes)
  for (int i = 0; i < PAD_I2C_CHUNK_SIZE; ++i) {
    i2cDummyData[i] = i;
  }
}

void updateLoadingData() {
  waterLoadingData.waterVolume = pulse / 660;
  waterLoadingData.timestamp = millis();

  airLoadingData.pressure = mpx5700.getPressureValue_kpa(1);
  airLoadingData.timestamp = millis();
}

void onI2CRequest() {
  // Only send already-prepared data, do not print or compute here
  Wire.write(i2cDummyData, PAD_I2C_CHUNK_SIZE);
  i2cRequestCount++;
}

void setup() {
  Serial.begin(115200);
  pinMode(AIR_DISTRIBUTOR_PIN, OUTPUT);
  pinMode(CYLINDER_PIN, OUTPUT);
  pinMode(CANCEL_PIN, OUTPUT);
  pinMode(WATER_VALVE_PIN, OUTPUT);

  pinMode(sensorPin, INPUT);

  // Set up interrupt to increment pulse each time the sensor detects water flowing,
  // allowing to deduce the volume that passed through
  attachInterrupt(digitalPinToInterrupt(sensorPin), increase, RISING);

  // Initialize I2C as slave
  Wire.begin(PAD_I2C_ADDRESS);
  Wire.setClock(I2C_BUS_SPEED);
  Wire.setWireTimeout(I2C_TIMEOUT);
  Wire.onRequest(onI2CRequest);
  Serial.print("I2C Slave ready at address 0x");
  Serial.println(PAD_I2C_ADDRESS, HEX);

  prepareDummyData();

  // ** Uncomment to use pressure sensor
  /*
  while (false == mpx5700.begin()) {
    Serial.println("i2c begin fail,please check connect!");
    delay(1000);
  }
  Serial.println("i2c begin success");

  mpx5700.setMeanSampleSize(5);

  */

  Serial.println("Setup done.");
}

void loop() {
  // Update loading data every 200 ms
  if (timer.expired()) {
    updateLoadingData();
  }

  // Update dummy data periodically if needed (example: every 100ms)
  if (millis() - lastDummyUpdate > 100) {
    prepareDummyData();
    lastDummyUpdate = millis();
  }

  // Print I2C data rate every second
  if (millis() - lastRatePrint > 1000) {
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
  // fillAir();
  // delay(1000);
  // stopAirFlow();
  // delay(1000); Delays are blocking i2C communication, so avoid them in
  // production code
}

void launchFlight() {
  if (launching_state == READY_TO_LAUNCH) {
    Serial.println("Launching.");
    // Launch the rocket
    digitalWrite(CYLINDER_PIN, HIGH);
    delay(2000);
    digitalWrite(CYLINDER_PIN, LOW);

    // launching_state = STAND_BY;
  } else {
    Serial.println("WARNING: Launch not ready yet.");
  }
}

void cancelFlight() {
  Serial.println("Cancelling flight...");
  launching_state = STAND_BY;

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
  if (waterFilled == false) {
    Serial.println("WARNING: Water hasn't been filled yet !");
    return;
  }

  digitalWrite(AIR_DISTRIBUTOR_PIN, HIGH);
  Serial.println("Starting to fill air...");

  launching_state = FILLING_AIR;

  airLoadingData.isLoading = true;
}

void fillWater() {
  // Starts to fill the rocket with water.

  if (waterFilled == true) {
    Serial.println("WARNING: Water has already been filled !");
  }
  digitalWrite(CANCEL_PIN, LOW);

  digitalWrite(WATER_VALVE_PIN, HIGH);
  Serial.println("Starting to fill water...");

  launching_state = FILLING_WATER;

  waterLoadingData.isLoading = true;
}

void stopWaterFlow() {
  // Stops the water flow
  digitalWrite(WATER_VALVE_PIN, LOW);

  Serial.println("Water valve closed");

  waterLoadingData.isLoading = false;

  launching_state = STAND_BY;
}

void stopAirFlow() {
  // Stops the air flow
  digitalWrite(AIR_DISTRIBUTOR_PIN, LOW);

  Serial.println("Air valve closed");

  airLoadingData.isLoading = false;

  launching_state = STAND_BY;
}

void increase() { pulse++; }
