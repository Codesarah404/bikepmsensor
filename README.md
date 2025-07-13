# bikepmsensor
This project presents a portable air quality monitoring system for bicycles. It measures PM2.5 and PM10 in real time and transmits the data via LoRaWAN, helping cyclists choose healthier routes based on current pollution levels.

This project was originally developed as part of a master's thesis at **HTW Berlin (University of Applied Sciences)** in 2025.

---

## 📡 Features

- Real-time PM2.5 / PM10 monitoring with HM330X
- GPS-based location via u-blox NEO-6M
- LoRaWAN transmission (TTN compatible)
- OLED display for live data (PM-values/risk based on WHO air quality limits)
- RGB-LED traffic light

---

## 🔧 Hardware Used

- Heltec WiFi LoRa 32 V2 (ESP32 + LoRa + OLED + charging)
- HM3303 fine dust sensor
- DHT22 temperature and humidity sensor
- u-blox NEO-6M GPS module
- RGB LED
- LiPo battery

