# 📦 Pharmaceutical Goods Monitoring System using IoT

This project is a comprehensive IoT solution designed to ensure the integrity and safety of pharmaceutical goods during transit. It provides real-time monitoring of critical environmental conditions, location, and handling, alerting users to any deviations.

---

### ⚙️ Hardware Architecture

The system employs a dual-microcontroller setup for robust data acquisition and communication:

#### Sensor Hub (STM32L476)
Responsible for collecting data from various sensors. It runs **FreeRTOS** for efficient task management and includes **CRC checks** for data integrity.

**STM32 Pinout:**
* **GPS Module:**
    * Vcc -> 3V3
    * GND -> GND
    * Rx -> PC12
    * Tx -> PD2
* **MQ-2 Smoke Sensor:**
    * A0 -> PC0
    * Vcc -> 5
    * GND -> GND
* **MPU6050 (Accelerometer/Gyroscope):**
    * Vcc -> 3V3
    * GND -> GND
    * SDA -> PB7
    * SCL -> PB6
* **DHT11/22 (Temperature/Humidity Sensor):**
    * Data -> PC10
    * Vcc -> 3v3
    * GND -> GND

#### Communication Gateway (ESP32)
Receives sensor data from the STM32 and publishes it to the cloud.

**ESP32 Pinout & Features:**
* **STM32 Communication (UART):**
    * STM32 (GND) -> ESP32 (GND)
    * RX2 (ESP32) -> PA9 (STM)
    * TX2 (ESP32) -> PA10 (STM)
* **Wi-Fi Provisioning Status LED:**
    * LED (+) -> D13
    * LED (-) -> Resistor Terminal (to ESP32 GND)
    * This LED on **GPIO13** indicates the status of Wi-Fi provisioning: it glows when provisioning starts and turns off upon successful completion.
* **Reset Button:**
    * Tactile Switch -> D27 & Vin
    * This button connected to **GPIO27** provides a hardware reset functionality, triggered on a positive edge (POSEDGE).

---

### 🌐 Communication & Cloud Integration

* **STM32 to ESP32:** Data is transmitted via **UART**.
* **ESP32 to Cloud:** Utilizes **Wi-Fi** to publish data to a **HiveMQ MQTT broker**.
* **Data Storage:** **InfluxDB** is used for efficient storage of time-series sensor data.
* **Monitoring & Alerts:** A **Node-RED** instance subscribes to the MQTT stream, displaying data on a **UI Dashboard** and sending **email alerts** when critical thresholds are breached (e.g., unusual temperature, smoke detection, or impact from acceleration data).

#### Data Packet Structure
The data sent from the STM32 to the ESP32 is a single comma-separated string, validated by a start marker (`AA`) and a CRC value.

> `AA,<temperature>,<humidity>,<accx>,<accy>,<accz>,<gyrx>,<gyry>,<gyrz>,<ppm>,<latitude>,<longitude>,<date>,<time>,<speed>,<crc_value>`