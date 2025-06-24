#include <Arduino.h>
#include <MemoryFree.h>
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

enum launchStates launchState = STAND_BY;

WaterLoadingData waterLoadingData;
AirLoadingData airLoadingData;

int sensorPin = 2;
volatile long pulse;

unsigned long launchTimestamp = 0; // Timestamp of the last launch
unsigned long cancelTimestamp = 0; // Timestamp of the last cancel

// Data rate counter for I2C requests
volatile unsigned long i2cRequestCount = 0;
volatile unsigned long i2cBytesWritten = 0;
volatile unsigned long i2cBytesReceived = 0;
unsigned long lastRatePrint = 0;
unsigned long lastRate = 0;
unsigned long lastI2CActivity = 0;

unsigned long startedFillingWater = 0;
unsigned long startedFillingAir = 0;

float zeroPressure = 0.0; // Zero reference for pressure sensor

float maxPressure = 0.0;                // Maximum pressure recorded air loading
unsigned long maxPressureTimestamp = 0; // Timestamp of maximum pressure

// Add a new pulse counter for tracking flow rate
volatile long flowRatePulse = 0;

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
void onI2CReceive(int numBytes);
void resetAfterLaunch();
void endCancelFlight();
void printI2CDataRate();
void resetFlowRatePulse();
void resetLaunchState();

const int BUFFER_SIZE = PAD_DATA_SEND_FREQUENCY * 1.5; // 1.5 seconds of data
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
#ifdef PAD_DEBUG
  long currentPulse = startedFillingWater > 0
                          ? (millis() - startedFillingWater) / 1000 * 660 /
                                4 // 4 seconds of water flow
                          : 0;
#else
  long currentPulse = pulse;
#endif
  waterLoadingData.waterVolume = currentPulse / 660.0;
  waterLoadingData.error = waterLoadingData.waterVolume - targetWaterVolume;
  // Ensure deltaTime is valid and reset flowRatePulse after calculation
  unsigned long deltaTime = waterLoadingData.timestamp > 0
                                ? millis() - waterLoadingData.timestamp
                                : 0;
  if (deltaTime >
      10) { // Minimum threshold to avoid division by very small values
    waterLoadingData.waterFlowRate =
        flowRatePulse / 660.0 / (deltaTime / 1000.0);
  } else {
    waterLoadingData.waterFlowRate =
        0; // Set flow rate to 0 if deltaTime is too small
  }
  resetFlowRatePulse(); // Reset the pulse counter after calculation
  waterLoadingData.timestamp = millis() - startedFillingWater;

#ifdef PAD_DEBUG
  airLoadingData.pressure =
      startedFillingAir > 0
          ? 100.0 + (millis() - startedFillingAir) / 1000.0 * 10.0
          : 0; // Simulated pressure
#else
  float sensorValue = mpx5700.getPressureValue_kpa(1);
  /*airLoadingData.pressure =
      sensorValue -
      zeroPressure; // Convert to relative pressure using zero reference*/
  airLoadingData.pressure = sensorValue - 100.0; // Adjusted for calibration
#endif
  airLoadingData.error = airLoadingData.pressure - targetAirPressure;
  airLoadingData.timestamp = millis() - startedFillingAir;
}

void onI2CRequest() {
  lastI2CActivity = millis(); // Update timestamp
  if (!padDataQueue.isEmpty()) {
    PadDataPacket packet;
    if (padDataQueue.dequeue(packet)) {
      // Serialize the entire packet as raw bytes
      size_t size = sizeof(PadDataPacket);

      Wire.write((uint8_t *)&packet, size);
      i2cBytesWritten += size; // Count bytes written
    }
  } else {
    PadDataPacket emptyPacket;
    memset(&emptyPacket, 0, sizeof(PadDataPacket));
    emptyPacket.type = PadDataPacket::NO_DATA;
    emptyPacket.launchState = launchState;
    size_t size = sizeof(PadDataPacket);
    Wire.write((uint8_t *)&emptyPacket, size);
    i2cBytesWritten += size; // Count bytes written
  }
}

void addWaterLoadingDataToQueue(const WaterLoadingData &data) {
  PadDataPacket packet;
  packet.launchState = launchState;
  packet.type = PadDataPacket::WATER_LOADING_DATA;
  packet.data.waterLoadingData = data;
  if (!padDataQueue.enqueue(packet)) {
    Serial.println("Queue is full. Dropping water loading data.");
  }
}

void addAirLoadingDataToQueue(const AirLoadingData &data) {
  PadDataPacket packet;
  packet.launchState = launchState;
  packet.type = PadDataPacket::AIR_LOADING_DATA;
  packet.data.airLoadingData = data;
  if (!padDataQueue.enqueue(packet)) {
    Serial.println("Queue is full. Dropping air loading data.");
  }
}

void sendLoadingData() {
  if (launchState == FILLING_WATER || launchState == FILLED_WATER) {
    // Add water loading data to the queue
    addWaterLoadingDataToQueue(waterLoadingData);
  } else if (launchState == FILLING_AIR || launchState == READY_TO_LAUNCH) {
    // Add air loading data to the queue
    addAirLoadingDataToQueue(airLoadingData);
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
  Wire.onReceive(onI2CReceive);
  Serial.print("I2C Slave ready at address 0x");
  Serial.println(PAD_I2C_ADDRESS, HEX);
}

void initializeMPX5700() {
#ifdef PAD_DEBUG
  Serial.println("[DEBUG] Initializing MPX5700 (Fake Mode)");
#else
  while (!mpx5700.begin()) {
    Serial.println("MPX5700 begin failed");
    delay(1000);
  }
  Serial.println("i2c begin success");
  mpx5700.setMeanSampleSize(5);
#endif
}

void setup() {
  Serial.begin(115200);

  initializePins();
  attachInterrupt(digitalPinToInterrupt(sensorPin), increase, RISING);
  resetLaunchState();

  initializeI2C();
  initializeMPX5700();

  // Read the initial pressure value as zero reference
  zeroPressure = mpx5700.getPressureValue_kpa(1);
  Serial.print("ZeroPressure: ");
  Serial.println(zeroPressure);

  Serial.println("PAD ready");
}

void loop() {

  static Timer dataRateTimer(1000);
  static Timer sensorCheckTimer(0); // Timer for checking sensors
  static Timer dataSendTimer(PAD_DATA_SEND_INTERVAL); // Timer for sending data

  // Print I2C data rate every second
  if (dataRateTimer.expired()) {
    printI2CDataRate();
  }

  if (sensorCheckTimer.expired()) {
    updateLoadingData();
    if (launchState == FILLING_AIR) {
      // Checks pressure sensor

      float currentAirPressure = airLoadingData.pressure;

      // Update maxPressure and maxPressureTimestamp
      if (currentAirPressure >= maxPressure) {
        maxPressure = currentAirPressure;
        maxPressureTimestamp = millis();
      }

      // Detect pressure drop
      if (millis() - maxPressureTimestamp > 3000) {
        Serial.println("Fallback: Pressure drop detected");
        launchState = READY_TO_LAUNCH;
        stopAirFlow();
      } else if (currentAirPressure >= targetAirPressure) {
        // Target air pressure reached. Now closing valve.
        launchState = READY_TO_LAUNCH;
        stopAirFlow();
      }
    }

    if (launchState == FILLING_WATER) {
      // Checks water flow sensor

      float currentWaterVolume = waterLoadingData.waterVolume;

      if (currentWaterVolume >= targetWaterVolume) {
        // Target water volume. Now closing valve.
        launchState = FILLED_WATER;
        stopWaterFlow();
      }
    }
  }

  if (launchState == LAUNCHING && millis() - launchTimestamp >= 1000) {
    // Wait for 1 second after launch to reset
    resetAfterLaunch();
  }

  if (launchState == CANCELLING && millis() - cancelTimestamp >= 20000) {
    // Wait for 20 seconds after cancel to end the procedure
    endCancelFlight();
  }

  if (dataSendTimer.expired()) {
    sendLoadingData();
  }

  // Check for I2C inactivity
  if (millis() - lastI2CActivity > 1000) {
    Serial.println("I2C reset...");
    Wire.end();
    initializeI2C();            // Reinitialize I2C
    lastI2CActivity = millis(); // Reset the timestamp after reinitialization
  }
}

void launchFlight() {
  if (launchState == READY_TO_LAUNCH) {
    Serial.println("Launching.");
    // Launch the rocket
    digitalWrite(CYLINDER_PIN, HIGH);
    launchState = LAUNCHING;
    launchTimestamp = millis(); // Record the launch time
  } else {
    Serial.println("Not ready yet.");
  }
}

void resetAfterLaunch() {
  if (launchState == LAUNCHING) {
    digitalWrite(CYLINDER_PIN, LOW);

    launchState = LAUNCHED;
  }
}

void cancelFlight() {
  Serial.println("Cancelling flight...");
  launchState = CANCELLING;
  cancelTimestamp = millis(); // Record the cancel time

  stopWaterFlow();
  stopAirFlow();

  // Cancels the flight by releasing the air / water contained in the rocket
  digitalWrite(CANCEL_PIN, HIGH);
}

void endCancelFlight() {
  // Ends the cancel flight procedure.
  digitalWrite(CANCEL_PIN, LOW);
  Serial.println("Flight cancelled.");
  launchState = CANCELLED;
}

void resetLaunchState() {
  // Resets the launch state to STAND_BY.
  Serial.println("Resetting launch state");
  launchState = STAND_BY;
  pulse = 0;                              // Reset the pulse count
  flowRatePulse = 0;                      // Reset the flow rate pulse count
  waterLoadingData = {0, 0.0, 0.0, 0.0};  // Reset water loading data
  airLoadingData = {0, 0.0, 0.0};         // Reset air loading data
  digitalWrite(CYLINDER_PIN, LOW);        // Ensure the cylinder is closed
  digitalWrite(AIR_DISTRIBUTOR_PIN, LOW); // Ensure air distributor is closed
  digitalWrite(WATER_VALVE_PIN, LOW);     // Ensure water valve is closed
  digitalWrite(CANCEL_PIN, LOW);          // Ensure cancel pin is low
  zeroPressure = mpx5700.getPressureValue_kpa(1);
  Serial.print("ZeroPressure: ");
  Serial.println(zeroPressure);

  Serial.println("Reset complete.");
}

void fillAir() {
  // Starts to fill the rocket with air.

  // Doesn't close the cancel_pin as it should already be closed.
  if (launchState != FILLED_WATER) {
    Serial.println("Water hasn't been filled yet !");
    return;
  }

  // Reset maxPressure and maxPressureTimestamp before starting air filling
  maxPressure = 0.0;
  maxPressureTimestamp = 0;

  // Closes the circuit.
  digitalWrite(CANCEL_PIN, LOW);

  digitalWrite(AIR_DISTRIBUTOR_PIN, HIGH);
  Serial.println("Starting to fill air...");

  launchState = FILLING_AIR;
  startedFillingAir = millis();
}

void fillWater() {
  // Starts to fill the rocket with water.

  if (launchState == FILLED_WATER) {
    Serial.println("Water has already been filled !");
  }

  // Resets the pulse to start counting again.
  pulse = 0;

  // Closes the circuit.
  digitalWrite(CANCEL_PIN, LOW);

  digitalWrite(WATER_VALVE_PIN, HIGH);
  Serial.println("Starting to fill water...");

  launchState = FILLING_WATER;

  startedFillingWater = millis(); // Start the debug timer
}

void stopWaterFlow() {
  // Stops the water flow
  digitalWrite(WATER_VALVE_PIN, LOW);

  Serial.println("Water valve closed");
}

void stopAirFlow() {
  // Stops the air flow
  digitalWrite(AIR_DISTRIBUTOR_PIN, LOW);

  Serial.println("Air valve closed");
}

void increase() {
  pulse++;
  flowRatePulse++; // Increment the flow rate pulse counter
}

void resetFlowRatePulse() { flowRatePulse = 0; }

void onI2CReceive(int numBytes) {
  lastI2CActivity = millis(); // Update timestamp
  Serial.print("I2C data received: ");
  Serial.print(numBytes);
  Serial.println(" bytes");

  // Print free memory
  Serial.print("Free memory: ");
  Serial.print(freeMemory());
  Serial.println(" bytes");

  if (numBytes < 1) {
    Serial.println("Received empty data");
    return;
  }

  char tempBuf[32];
  size_t copyLen =
      (numBytes < (int)(sizeof(tempBuf))) ? numBytes : (sizeof(tempBuf) - 1);
  Wire.readBytes(tempBuf, copyLen);
  tempBuf[copyLen] = '\0'; // Null-terminate the string

  // Trim leading and trailing whitespace manually
  char *start = tempBuf;
  while (*start == ' ' || *start == '\t') {
    ++start;
  }

  char *end = start + strlen(start) - 1;
  while (end > start && (*end == ' ' || *end == '\t')) {
    *end = '\0';
    --end;
  }

  if (strlen(start) == 0) {
    Serial.println("Received empty or invalid command");
    return;
  }

  Serial.print("Received command: ");
  Serial.println(start);

  if (strcmp(start, "launch") == 0) {
    if (launchState == READY_TO_LAUNCH) {
      launchFlight();
    } else {
      Serial.println("Cannot launch: not ready.");
    }
  } else if (strcmp(start, "cancel") == 0) {
    if (launchState != STAND_BY) {
      cancelFlight();
    } else {
      Serial.println("Cannot cancel: no flight in progress.");
    }
  } else if (strncmp(start, "fill_air", 8) == 0) {
    if (launchState == FILLED_WATER) {
      targetAirPressure = atof(start + 9);

      fillAir();
    } else {
      Serial.println("Cannot fill air: water not filled yet.");
    }
  } else if (strncmp(start, "fill_water", 10) == 0) {
    if (launchState == STAND_BY) {
      targetWaterVolume = atof(start + 11);
      fillWater();
    } else {
      Serial.println("Cannot fill water: already in progress.");
    }
  } else if (strcmp(start, "send_data") == 0) {
    addAirLoadingDataToQueue(airLoadingData);
    addWaterLoadingDataToQueue(waterLoadingData);
    Serial.println("Loading data sent.");
  } else if (strcmp(start, "reset") == 0) {
    resetLaunchState();
    Serial.println("Launch state reset to STAND_BY.");
  } else {
    Serial.print("Unknown command: ");
    Serial.println(start);
  }
}

void printI2CDataRate() {
  lastRate = i2cRequestCount;
  i2cRequestCount = 0;
  unsigned long bytesPerSecond = lastRate * PAD_I2C_CHUNK_SIZE;
  unsigned long writtenBytes = i2cBytesWritten;
  unsigned long receivedBytes = i2cBytesReceived;
  i2cBytesWritten = 0;  // Reset counter
  i2cBytesReceived = 0; // Reset counter

  Serial.print("I2C requests per second: ");
  Serial.print(lastRate);
  Serial.print(", Data rate: ");
  Serial.print(bytesPerSecond);
  Serial.print(" B/s, Bytes written: ");
  Serial.print(writtenBytes);
  Serial.print(" B, Bytes received: ");
  Serial.println(receivedBytes);

  // Print free memory
  Serial.print("Free memory: ");
  Serial.print(freeMemory());
  Serial.println(" bytes");

  // Also print the current launch state
  Serial.print("Current launch state: ");
  Serial.println(launchState);

  Serial.print("Current air pressure: ");
  Serial.println(airLoadingData.pressure);
  Serial.print("Target air pressure: ");
  Serial.println(targetAirPressure);
}
