## 📦 Requirements

### Hardware

* ESP32 (tested with ESP32-S3-WROOM)
* **TF02 LiDAR** sensor

  * TF02 **TX → ESP32 GPIO17 (RX2)**
  * TF02 **VCC → 5V**, **GND → GND**
* **BN-220 GPS** module

  * GPS **TX → ESP32 GPIO16 (RX1)**
  * GPS **VCC → 3.3V**, **GND → GND**
* LED Matrix (MAX7219, 32×8 modules chained)

  * **CS → GPIO5**
  * **CLK → GPIO6**
  * **DIN → GPIO4**

### Software

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Arduino IDE → Boards Manager → Install **ESP32 by Espressif Systems**
3. Required libraries (install via Arduino Library Manager):

   * **MD\_Parola**
   * **MD\_MAX72xx**
   * **WiFi** (built-in with ESP32)
   * **WebServer** (built-in with ESP32)
   * **Preferences** (built-in with ESP32)

---

## 🚀 Usage

1. Clone this repository and open the project in Arduino IDE.
2. Select **Board: ESP32 Dev Module** (or your ESP32 variant).
3. Flash the sketch.
4. The ESP32 starts as a Wi-Fi AP (default SSID: `ESP32-WLAN`, PW: `12345678`).
5. Open a browser and go to `192.168.4.1` to access the WebUI.

---

## 🌐 WebUI Features

* **Live Data**: speed, satellites, distance, system status
* **Toggle system on/off**
* **Change Wi-Fi settings**
* **Send custom messages to LED display**
* **Debug terminal**

---

## ⚠️ Notes

* The system only activates when:

  * System is ON
  * GPS has ≥ 4 satellites
  * Speed ≥ 15 km/h
* Safe distance is calculated using both physics-based and rule-of-thumb formulas.
* Custom messages are shown for 10 seconds.

This Readme was created using AI
