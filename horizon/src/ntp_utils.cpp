#include "ntp_utils.h"
#include <Arduino.h>

void initNtp(NTPClient &ntpClient) {
  ntpClient.update(); // Update time immediately
  Serial.print("NTP Time: ");
  Serial.print(ntpClient.getFormattedTime());
  Serial.print(", Epoch: ");
  Serial.println(ntpClient.getEpochTime());
}
