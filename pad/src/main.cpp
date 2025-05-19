#include <Arduino.h>

#include "DFRobot_MPX5700.h"
#define I2C_ADDRESS 0x19
DFRobot_MPX5700 mpx5700(&Wire, I2C_ADDRESS);
void setup() {
  Serial.begin(115200);
  while (false == mpx5700.begin()) {
    Serial.println("i2c begin fail,please chack connect!");
    delay(1000);
  }
  Serial.println("i2c begin success");

  // In order to smooth data, set to take the mean value based on xx (quantity)
  // adc data, if not set, the system will automatically get the mean value of 5
  // samples.
  mpx5700.setMeanSampleSize(/*Sample Total*/ 5);
}

void loop() {
  Serial.print("Pressure Value: ");
  // Get the current ambient air pressure, set whether to enable calibration, 1
  // for calibration required, 0 for no calibration required
  Serial.print(mpx5700.getPressureValue_kpa(1));
  Serial.println(" kpa");
  delay(1000);
}
