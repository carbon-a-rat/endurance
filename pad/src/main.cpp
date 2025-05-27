#include <Arduino.h>

#include "DFRobot_MPX5700.h"
#define I2C_ADDRESS 0x16

#define CANCEL_PIN 13
#define AIR_DISTRIBUTOR_PIN 12
#define CYLINDER_PIN 11
#define WATER_VALVE_PIN 10

DFRobot_MPX5700 mpx5700(&Wire, I2C_ADDRESS);

const float targetPressure = 200.0; // in kPa

bool fillingAir = false;

void setup() {
  Serial.begin(115200);
  pinMode(AIR_DISTRIBUTOR_PIN, OUTPUT);
  pinMode(CYLINDER_PIN, OUTPUT);
  pinMode(CANCEL_PIN, OUTPUT);

  while (false == mpx5700.begin()) {
    Serial.println("i2c begin fail,please check connect!");
    delay(1000);
  }
  Serial.println("i2c begin success");
  
  mpx5700.setMeanSampleSize(/*Sample Total*/ 5);
}

void loop() {
  // Get the current ambient air pressure:
  // set whether to enable calibration, 1
  //  for calibration required, 0 for no calibration required
  float pressureValue = mpx5700.getPressureValue_kpa(1);

  Serial.print("Current pressure: ");
  Serial.print(pressureValue);
  Serial.println(" kpa");
  
  Serial.print("Target pressure: ");
  Serial.print(targetPressure);
  Serial.println(" kpa");

  // While the pressure inside the rocket is < to the target pressure (600 kPa):
  if (fillingAir == true) {
    if (pressureValue >= targetPressure) {
      Serial.println("Target pressure reached: should be stopping filling.");
      digitalWrite(AIR_DISTRIBUTOR_PIN, LOW);
    }
    else {
      // Fill the rocket with air
      digitalWrite(AIR_DISTRIBUTOR_PIN, HIGH);
    }
    
  }
  delay(100);
}

void cancelFlight() {
  // Stops the air flow
  fillingAir = false;
  digitalWrite(AIR_DISTRIBUTOR_PIN, LOW);

  // Cancels the flight by releasing the air / water contained in the rocket
  digitalWrite(CANCEL_PIN, HIGH);
}

void fillAir() {
  // Start to fill the rocket with air.

  // Close the CANCEL_PIN
  digitalWrite(CANCEL_PIN, LOW);
  fillingAir = true;
}
