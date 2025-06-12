#pragma once
#include "pocketbase.hpp"
#include "state.h"
#include <NTPClient.h>
#include <Timer.h>

void initPocketbase(PocketbaseArduino &pocketbaseConnection,
                    PocketbaseState &pocketbaseState);
void pocketbaseLoop(PocketbaseState &pocketbaseState,
                    PocketbaseArduino &pocketbaseConnection,
                    NTPClient &ntpClient);
String epochToPocketbaseTime(unsigned long epochTime);
void heartbeatPocketbase(PocketbaseArduino &pocketbaseConnection,
                         PocketbaseState &pocketbaseState,
                         NTPClient &ntpClient);