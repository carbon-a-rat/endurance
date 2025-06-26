#include "request_builders.h"

String processQueueBatch(int maxBatchSize) {
  String batchRequest;
  batchRequest.reserve(42000); // Pre-allocate memory to prevent reallocations
  int count = 0;

  while (!requestQueue.isEmpty() && count < maxBatchSize) {
    QueuedRequestWrapper request;
    if (requestQueue.dequeue(request)) {
      String requestStr;
      switch (request.type) {
      case FLIGHT_DATA:
        requestStr = buildFlightDataRequest(request.request.flightDataRequest);
        break;
      case WATER_LOADING_DATA:
        requestStr = buildWaterLoadingDataRequest(
            request.request.waterLoadingDataRequest);
        break;
      case AIR_LOADING_DATA:
        requestStr =
            buildAirLoadingDataRequest(request.request.airLoadingDataRequest);
        break;
      case LAUNCHER_DATA:
        requestStr =
            buildLauncherDataRequest(request.request.launcherDataRequest);
        break;
      case HEARTBEAT:
        requestStr = buildHeartbeatRequest(request.request.heartbeatRequest);
        break;
      case LAUNCH_UPDATE:
        requestStr =
            buildLaunchUpdateRequest(request.request.launchUpdateRequest);
        break;
      case LOG:
        requestStr = buildLogRequest(request.request.logRequest);
        break;
      default:
        Serial.println("Unknown request type");
        continue; // Skip unknown request types
      }
      if (!requestStr.isEmpty()) {
        if (count > 0) {
          batchRequest += ",";
        }
        batchRequest += requestStr;
        count++;
      } else {
        Serial.print("Failed to build request string, request type: ");
        Serial.println(request.type);
      }
    }
  }

  return batchRequest;
}