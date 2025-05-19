#include <Adafruit_DPS310.h>
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
    sensor.settings.accelRange = 16;       // ±16 g
    sensor.settings.accelSampleRate = 104; // 104 Hz
    sensor.settings.gyroRange = 245;       // ±245 dps
    sensor.settings.gyroSampleRate = 104;  // 104 Hz
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

class BarometerSensor {
private:
  Adafruit_DPS310 sensor;
  uint8_t i2cAddress;

public:
  BarometerSensor(uint8_t address = 0x77) : sensor(), i2cAddress(address) {}

  bool initialize() {
    if (!sensor.begin_I2C()) {
      return false;
    }
    sensor.setMode(DPS310_CONT_PRESTEMP);
    sensor.configurePressure(DPS310_128HZ, DPS310_16SAMPLES);
    sensor.configureTemperature(DPS310_128HZ, DPS310_16SAMPLES);
    return true;
  }

  void read(float &pressure, float &temperature, float &altitude) {
    sensors_event_t tempEvent, pressureEvent;
    while (!sensor.getEvents(&tempEvent, &pressureEvent)) {
    }
    temperature = tempEvent.temperature;
    pressure = pressureEvent.pressure;
    altitude = sensor.readAltitude(1013.25); // Sea level pressure in hPa
  }

  uint8_t getI2CAddress() const { return i2cAddress; }
};