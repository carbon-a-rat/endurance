struct FlightData {
  long unsigned timestamp;
  float pressure;
  float temperature;
  float altitude;
  float ax, ay, az;
  float gx, gy, gz;
  float batteryLevel;
  bool isDeployed;
  bool isFlying;
  bool isLanded;
};

int dataSendFrequency = 32; // 32 Hz
long dataSendDelay = 1000 / dataSendFrequency;
