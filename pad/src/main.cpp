#include <Arduino.h>

#include "DFRobot_MPX5700.h"
#define I2C_ADDRESS 0x16

#define CANCEL_PIN 13
#define AIR_DISTRIBUTOR_PIN 12
#define CYLINDER_PIN 11
#define WATER_VALVE_PIN 10

DFRobot_MPX5700 mpx5700(&Wire, I2C_ADDRESS);

const float targetAirPressure = 200.0; // in kPa
float targetWaterVolume = 0.5; // in L

enum launchingStates {
  STAND_BY,
  FILLING_AIR,
  FILLING_WATER,
  READY_TO_LAUNCH
};

enum launchingStates launching_state = STAND_BY;

int sensorPin = 2;
volatile long pulse;

bool waterFilled;
bool airFilled;

// Forward declarations
void increase();  
void stopWaterFlow();
void stopAirFlow();

void setup() {
  Serial.begin(115200);
  pinMode(AIR_DISTRIBUTOR_PIN, OUTPUT);
  pinMode(CYLINDER_PIN, OUTPUT);
  pinMode(CANCEL_PIN, OUTPUT);

  pinMode(sensorPin, INPUT);

  // Set up pulse incremental
  attachInterrupt(digitalPinToInterrupt(sensorPin), increase, RISING);

  // ** Uncomment to use pressure sensor
  /*
  while (false == mpx5700.begin()) {
    Serial.println("i2c begin fail,please check connect!");
    delay(1000);
  }
  Serial.println("i2c begin success");
  
  mpx5700.setMeanSampleSize(5);

  */
}

void loop() {
  switch (launching_state) {
    case STAND_BY:
      break;
    case FILLING_AIR:
      // air filling logic

      // Get the current ambient air pressure:
      // set whether to enable calibration, 1
      //  for calibration required, 0 for no calibration required
      float currentAirPressure = mpx5700.getPressureValue_kpa(1);

      Serial.print(currentAirPressure/targetAirPressure*100);
      Serial.println(" % filled.");

      if (currentAirPressure >= targetAirPressure) {
        Serial.println("Air target pressure reached. Closing valve.");
        digitalWrite(AIR_DISTRIBUTOR_PIN, LOW);
        airFilled = true;
        Serial.println("Air OK.");
      }
      break;

    case FILLING_WATER:
      // water filling logic

      // /!\ WARNING: CHECK THIS VALUE:
      // 1L = 660 pulses
      float currentWaterVolume = pulse / 660;
      Serial.print(currentWaterVolume/targetWaterVolume*100);
      Serial.println(" % filled.");

      if (currentWaterVolume >= targetWaterVolume) {
        // Reached desired water volume

        Serial.println("Water target volume reached. Now closing valve.");
        digitalWrite(WATER_VALVE_PIN, LOW);

        waterFilled = true;
        Serial.println("Water OK.");
      }
      break;
  }
  delay(100);
}

void launchFlight() {
  if (launching_state == READY_TO_LAUNCH) {
    Serial.println("Launching.");
    // Launch the rocket
    digitalWrite(CYLINDER_PIN, HIGH);
    delay(2000);
    digitalWrite(CYLINDER_PIN, LOW);
  }
  else {
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

  launching_state = FILLING_AIR;
  digitalWrite(AIR_DISTRIBUTOR_PIN, HIGH);
  Serial.println("Starting to fill air...");
}

void fillWater() {
  // Starts to fill the rocket with water.

  if (waterFilled == true) {
    Serial.println("WARNING: Water has already been filled !")
  }
  digitalWrite(CANCEL_PIN, LOW);

  launching_state = FILLING_WATER;

  digitalWrite(WATER_VALVE_PIN, HIGH);
  Serial.println("Starting to fill water...");
}

void stopWaterFlow() {
  // Stops the water flow
  digitalWrite(WATER_VALVE_PIN, LOW);

  launching_state = STAND_BY;
}

void stopAirFlow() {
  // Stops the air flow
  digitalWrite(AIR_DISTRIBUTOR_PIN, LOW);

  launching_state = STAND_BY;
}

void increase() {
  pulse++;
}