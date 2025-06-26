#include "enqueue_functions.h"
#include "queue_logic.h"

// Example implementation for enqueueing flight data
void enqueueFlightData(const FlightData &data) {
  if (requestQueue.isFull()) {
    Serial.println("Request queue is full. Dropping flight data.");
    return;
  }

  QueuedRequestWrapper *wrapper =
      (QueuedRequestWrapper *)calloc(1, sizeof(QueuedRequestWrapper));
  wrapper->request.flightDataRequest.data = data;
  wrapper->type = FLIGHT_DATA;

  requestQueue.enqueue(*wrapper);
}

void enqueueWaterLoadingData(const WaterLoadingData &data) {
  if (requestQueue.isFull()) {
    Serial.println("Request queue is full. Dropping water loading data.");
    return;
  }

  QueuedRequestWrapper *wrapper =
      (QueuedRequestWrapper *)calloc(1, sizeof(QueuedRequestWrapper));
  wrapper->request.waterLoadingDataRequest.data = data;
  wrapper->type = WATER_LOADING_DATA;

  requestQueue.enqueue(*wrapper);
}

void enqueueAirLoadingData(const AirLoadingData &data) {
  if (requestQueue.isFull()) {
    Serial.println("Request queue is full. Dropping air loading data.");
    return;
  }

  QueuedRequestWrapper *wrapper =
      (QueuedRequestWrapper *)calloc(1, sizeof(QueuedRequestWrapper));
  wrapper->request.airLoadingDataRequest.data = data;
  wrapper->type = AIR_LOADING_DATA;

  requestQueue.enqueue(*wrapper);
}

void enqueueLauncherData(const LauncherData &data) {
  if (requestQueue.isFull()) {
    Serial.println("Request queue is full. Dropping launcher data.");
    return;
  }

  QueuedRequestWrapper *wrapper =
      (QueuedRequestWrapper *)calloc(1, sizeof(QueuedRequestWrapper));
  wrapper->request.launcherDataRequest.data = data;
  wrapper->type = LAUNCHER_DATA;

  requestQueue.enqueue(*wrapper);
}

void enqueueHeartbeat(const Heartbeat &data) {
  if (requestQueue.isFull()) {
    Serial.println("Request queue is full. Dropping heartbeat data.");
    return;
  }

  QueuedRequestWrapper *wrapper =
      (QueuedRequestWrapper *)calloc(1, sizeof(QueuedRequestWrapper));
  if (!wrapper) {
    Serial.println("Memory allocation failed for heartbeat request wrapper.");
    Serial.println("Free PS RAM: " + String(ESP.getFreePsram()));
    Serial.println("Free heap: " + String(ESP.getFreeHeap()));
    return;
  }
  wrapper->request.heartbeatRequest.data = data;

  wrapper->type = HEARTBEAT;

  requestQueue.enqueue(*wrapper);
}

void enqueueLaunchUpdate(const LaunchUpdate &data) {
  if (requestQueue.isFull()) {
    Serial.println("Request queue is full. Dropping launch update data.");
    return;
  }

  QueuedRequestWrapper *wrapper =
      (QueuedRequestWrapper *)calloc(1, sizeof(QueuedRequestWrapper));
  wrapper->request.launchUpdateRequest.data = data;
  wrapper->type = LAUNCH_UPDATE;

  requestQueue.enqueue(*wrapper);
}

void enqueueLog(const LauncherLog &data) {
  if (requestQueue.isFull()) {
    Serial.println("Request queue is full. Dropping log data.");
    return;
  }

  QueuedRequestWrapper *wrapper =
      (QueuedRequestWrapper *)calloc(1, sizeof(QueuedRequestWrapper));
  wrapper->request.logRequest.data = data;
  wrapper->type = LOG;

  requestQueue.enqueue(*wrapper);
}