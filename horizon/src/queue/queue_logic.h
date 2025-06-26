#pragma once
#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "core.h"

// Enum for request types
enum RequestType {
  AIR_LOADING_DATA,
  WATER_LOADING_DATA,
  FLIGHT_DATA,
  LAUNCHER_DATA,
  HEARTBEAT,
  LAUNCH_UPDATE,
  LOG
};

// Structure for a queued request
struct BaseQueuedRequest {
  RequestType type;
  uint8_t priority; // Optional: Higher value = higher priority
};

struct AirLoadingDataRequest : public BaseQueuedRequest {
  AirLoadingData data; // Data specific to air loading
  AirLoadingDataRequest() {
    type = AIR_LOADING_DATA;
    priority = 1; // Default priority
  }
};
struct WaterLoadingDataRequest : public BaseQueuedRequest {
  WaterLoadingData data; // Data specific to water loading
  WaterLoadingDataRequest() {
    type = WATER_LOADING_DATA;
    priority = 1; // Default priority
  }
};

struct FlightDataRequest : public BaseQueuedRequest {
  FlightData data; // Data specific to flight
  FlightDataRequest() {
    type = FLIGHT_DATA;
    priority = 1; // Default priority
  }
};

struct LauncherDataRequest : public BaseQueuedRequest {
  LauncherData data; // Data specific to launcher
  LauncherDataRequest() {
    type = LAUNCHER_DATA;
    priority = 1; // Default priority
  }
};

struct HeartbeatRequest : public BaseQueuedRequest {
  Heartbeat data; // Data specific to heartbeat
  HeartbeatRequest() {
    type = HEARTBEAT;
    priority = 1; // Default priority
  }
};

struct LaunchUpdateRequest : public BaseQueuedRequest {
  LaunchUpdate data; // Data specific to launch updates
  LaunchUpdateRequest() {
    type = LAUNCH_UPDATE;
    priority = 1; // Default priority
  }
};
struct LogRequest : public BaseQueuedRequest {
  LauncherLog data; // Data specific to logs
  LogRequest() {
    type = LOG;
    priority = 1; // Default priority
  }
};

// Define a union to hold all possible request types
union QueuedRequestUnion {
  AirLoadingDataRequest airLoadingDataRequest;
  WaterLoadingDataRequest waterLoadingDataRequest;
  FlightDataRequest flightDataRequest;
  LauncherDataRequest launcherDataRequest;
  HeartbeatRequest heartbeatRequest;
  LaunchUpdateRequest launchUpdateRequest;
  LogRequest logRequest;

  QueuedRequestUnion() {}
  ~QueuedRequestUnion() {}
};

struct QueuedRequestWrapper {
  QueuedRequestUnion request;
  RequestType type;

  QueuedRequestWrapper() : type(RequestType::LOG) {}
  ~QueuedRequestWrapper() {}

  uint8_t priority() {
    switch (type) {
    case AIR_LOADING_DATA:
      return request.airLoadingDataRequest.priority;
    case WATER_LOADING_DATA:
      return request.waterLoadingDataRequest.priority;
    case FLIGHT_DATA:
      return request.flightDataRequest.priority;
    case LAUNCHER_DATA:
      return request.launcherDataRequest.priority;
    case HEARTBEAT:
      return request.heartbeatRequest.priority;
    case LAUNCH_UPDATE:
      return request.launchUpdateRequest.priority;
    case LOG:
      return request.logRequest.priority;
    default:
      return 0; // Default priority if type is unknown
    }
  }
  // Custom copy assignment operator
  QueuedRequestWrapper &operator=(const QueuedRequestWrapper &other) {
    if (this != &other) {
      memcpy(&request, &other.request, sizeof(QueuedRequestUnion));
      type = other.type;
    }
    return *this;
  }
};

// Circular buffer for the queue
const size_t QUEUE_SIZE = 50;
struct RequestQueue {
  QueuedRequestWrapper buffer[QUEUE_SIZE];
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;

  bool isFull() { return count == QUEUE_SIZE; }
  bool isEmpty() { return count == 0; }

  bool enqueue(const QueuedRequestWrapper &request) {
    if (isFull())
      return false;
    buffer[head] = request;
    head = (head + 1) % QUEUE_SIZE;
    count++;
    return true;
  }

  bool dequeue(QueuedRequestWrapper &request) {
    if (isEmpty())
      return false;

    // Find the highest-priority request
    size_t highestPriorityIndex = tail;
    for (size_t i = 0; i < count; i++) {
      size_t index = (tail + i) % QUEUE_SIZE;
      if (buffer[index].priority() > buffer[highestPriorityIndex].priority()) {
        highestPriorityIndex = index;
      }
    }

    // Dequeue the highest-priority request
    request = buffer[highestPriorityIndex];

    // Shift elements to fill the gap
    for (size_t i = highestPriorityIndex; i != head; i = (i + 1) % QUEUE_SIZE) {
      size_t nextIndex = (i + 1) % QUEUE_SIZE;
      buffer[i] = buffer[nextIndex];
    }

    head = (head + QUEUE_SIZE - 1) % QUEUE_SIZE;
    count--;
    return true;
  }
};

extern RequestQueue requestQueue;