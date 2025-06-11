#include <Arduino.h>
#include <Wire.h>
#include <core.h>
#include <Timer.h>

#include "DFRobot_MPX5700.h"
#define AIR_SENSOR_I2C_ADDRESS 0x16

#define CANCEL_PIN 13
#define AIR_DISTRIBUTOR_PIN 12
#define CYLINDER_PIN 11
#define WATER_VALVE_PIN 10

DFRobot_MPX5700 mpx5700(&Wire, AIR_SENSOR_I2C_ADDRESS);

const float targetAirPressure = 300.0; // in kPa
float targetWaterVolume = 0.5;         // in L

enum launchingStates { STAND_BY, FILLING_AIR, FILLING_WATER, READY_TO_LAUNCH };

enum launchingStates launchingState = STAND_BY;

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

Timer dataUpdateTimer(200);
Timer sensorCheckTimer(200);

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

void prepareDummyData() {
  // Fill dummy data (for now, just incrementing bytes)
  for (int i = 0; i < PAD_I2C_CHUNK_SIZE; ++i) {
    i2cDummyData[i] = i;
  }
}

void updateLoadingData() {
  long currentPulse = pulse;
  waterLoadingData.waterVolume = currentPulse / 660.0;
  waterLoadingData.timestamp = millis();

  airLoadingData.pressure = mpx5700.getPressureValue_kpa(1);
  airLoadingData.timestamp = millis();

  Serial.println(airLoadingData.pressure);
}

void onI2CRequest() {
  return;
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
  //Wire.begin(PAD_I2C_ADDRESS);
  //Wire.setClock(I2C_BUS_SPEED);
  //Wire.setWireTimeout(I2C_TIMEOUT);
  //Wire.onRequest(onI2CRequest);
  //Serial.print("I2C Slave ready at address 0x");
  //Serial.println(PAD_I2C_ADDRESS, HEX);

  //prepareDummyData();

  while (false == mpx5700.begin()) {
    Serial.println("i2c begin fail,please check connect!");
    delay(1000);
  }
  Serial.println("i2c begin success");

  mpx5700.setMeanSampleSize(5);
  
  Serial.println("Setup done. Filling air in 5 sec");
  delay(5000);
  fillAir();
  
}

void loop() {
  // Update loading data every 200 ms
  if (dataUpdateTimer.expired()) {
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

  if (sensorCheckTimer.expired()) {
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
