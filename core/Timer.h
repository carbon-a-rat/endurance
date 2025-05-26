#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

class Timer {
public:
  Timer(unsigned long intervalMs = 0) : interval(intervalMs), last(0) {}

  void setInterval(unsigned long intervalMs) { interval = intervalMs; }
  void reset() { last = millis(); }
  bool expired() {
    unsigned long now = millis();
    if (now - last >= interval) {
      last = now;
      return true;
    }
    return false;
  }

private:
  unsigned long interval;
  unsigned long last;
};

#endif // TIMER_H
