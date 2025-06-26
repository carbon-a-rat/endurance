#pragma once
#include "pocketbase.hpp"
#include "state.h"
#include <NTPClient.h>
#include <Timer.h>

void initPocketbase();
String epochToPocketbaseTime(unsigned long epochTime);
void onLaunchUpdate(SubscriptionEvent &ev, void *ctx);