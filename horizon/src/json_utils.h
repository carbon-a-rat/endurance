#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

struct psramAllocator {
  static void *allocate(size_t size) { return ps_malloc(size); }

  static void deallocate(void *ptr) { free(ptr); }

  static void *reallocate(void *ptr, size_t size) {
    if (ptr == nullptr) {
      return ps_malloc(size);
    }
    void *newPtr = ps_malloc(size);
    if (newPtr) {
      memcpy(newPtr, ptr, size);
      free(ptr);
    }
    return newPtr;
  }
};

// typedef BasicJsonDocument<psramAllocator> JsonDocumentPSRAM;
//  No psram on the ESP32 pico kit