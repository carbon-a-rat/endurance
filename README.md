# endurance
to-the-mars team's rocket launcher and probe system

## Embedded controllers used
- Feather HUZZAH ESP86
  - Documentation: https://learn.adafruit.com/adafruit-feather-huzzah-esp8266/overview
- Arduino UNO
  - Documentation: https://www.arduino.cc/en/Guide/ArduinoUno

## Project Overview

This system is composed of multiple microcontrollers working together for real-time telemetry and autonomous launch control of a water rocket. 

### 🗺️ Board Roles & Codenames

| Codename   | Hardware     | Role                                                                 |
|------------|--------------|----------------------------------------------------------------------|
| `probe`    | ESP8266      | Onboard the water rocket. Collects sensor data and communicates bidirectionally with `gateway` via ESP-NOW. |
| `gateway`  | ESP8266      | Relay node. Exchanges data with `probe` via ESP-NOW, and forwards information to `horizon` via I2C. |
| `horizon`  | ESP8266      | Central controller (master node). Connected to Wi-Fi; acts as an HTTPS client to send data to the control server. Receives Server-Sent Events (SSE) |
| `pad`      | Arduino Uno  | Launcher controller. Manages water/air loading and triggers the rocket launch sequence. |

---

*Due to resource constraints, the `gateway` and `horizon` features are splitted across two ESP8266 boards. With an ESP32, this could be done in a single board.*



### 🔄 Communication Diagram

```text
       [ Rocket ]
        ┌──────┐
        │probe │ ◄───► ESP-NOW ◄───►
        └──────┘               ┌────────┐
                               │gateway │
                               └────────┘
                                   │
                                  I2C
                                   │
                               ┌────────┐
                               │horizon │ ──► HTTPS API Client
                               └────────┘       (death-star-command-center)
                                   │
                                  I2C
                                   │
                               ┌─────┐
                               │ pad │ ←─── Controls pneumatics, valves, etc.
                               └─────┘
