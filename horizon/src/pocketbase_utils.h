#pragma once
#include "pocketbase.hpp"
#include "state.h"

void initPocketbase(PocketbaseArduino &pocketbaseConnection,
                    PocketbaseState &pocketbaseState);
void pocketbaseLoop(PocketbaseState &pocketbaseState,
                    PocketbaseArduino &pocketbaseConnection);
