#include <Arduino.h>
#include <LSM6DS3.h>

class AccGyroSensor {
private:
  LSM6DS3 sensor;
  uint8_t i2cAddress;

public:
  AccGyroSensor(uint8_t address = 0x6A)
      : sensor(I2C_MODE, address), i2cAddress(address) {}

  bool initialize() {
    sensor.settings.accelRange = 4;  // ±4g
    sensor.settings.gyroRange = 245; // ±245 dps
    return sensor.begin() == 0;
  }

  void read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    ax = sensor.readFloatAccelX();
    ay = sensor.readFloatAccelY();
    az = sensor.readFloatAccelZ();
    gx = sensor.readFloatGyroX();
    gy = sensor.readFloatGyroY();
    gz = sensor.readFloatGyroZ();
  }

  uint8_t getI2CAddress() const { return i2cAddress; }
};