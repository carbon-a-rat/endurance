#include <Arduino.h>

#include "DFRobot_MPX5700.h"
#define I2C_ADDRESS 0x19

#define AIR_DISTRIBUTOR_PIN 13
#define CYLINDER_PIN 12
#define CANCEL_PIN 11
#define WATER_VALVE_PIN 10

// DFRobot_MPX5700 mpx5700(&Wire, I2C_ADDRESS);
void setup() {
  Serial.begin(115200);
  pinMode(AIR_DISTRIBUTOR_PIN, OUTPUT);
  pinMode(CYLINDER_PIN, OUTPUT);
  pinMode(CANCEL_PIN, OUTPUT);

  // while (false == mpx5700.begin()) {
  //   Serial.println("i2c begin fail,please check connect!");
  //   delay(1000);
  // }*

  // mpx5700.setMeanSampleSize(/*Sample Total*/ 5);
}

void loop() {
  // output on the digital pins
  digitalWrite(AIR_DISTRIBUTOR_PIN, HIGH);
  digitalWrite(CYLINDER_PIN, HIGH);
  digitalWrite(CANCEL_PIN, HIGH);
  delay(1000);
  digitalWrite(AIR_DISTRIBUTOR_PIN, LOW);
  digitalWrite(CYLINDER_PIN, LOW);
  digitalWrite(CANCEL_PIN, LOW);

  // Serial.print("Pressure Value: ");
  //  Get the current ambient air pressure, set whether to enable calibration, 1
  //  for calibration required, 0 for no calibration required
  // Serial.print(mpx5700.getPressureValue_kpa(1));
  // Serial.println(" kpa");
  delay(1000);
}
