# Custom-ELRS
This collection of C/C++ files constitutes the core firmware structure for a highly customized, open-source ExpressLRS (ELRS) radio transmitter module. It is specifically tailored for the ESP32-C6 (a low-power Wi-Fi/Bluetooth chip) and the SX1280 2.4GHz LoRa radio.
📡 ExpressLRS (ELRS) Integrated RC Transmitter for FPV/Drones
This project represents the foundational firmware for a custom, integrated ExpressLRS (ELRS) Transmitter (TX) solution. It utilizes a Nordic-like architecture where the ESP32-C6 microcontroller not only manages the system logic and user configuration but also directly reads the raw stick inputs (e.g., from an integrated RC controller's gimbals) via its Analog-to-Digital Converter (ADC). This data is then immediately fed into the ELRS protocol stack for transmission using the Semtech SX1280 2.4GHz LoRa radio chip.

The goal is to create a high-performance, low-latency, and long-range control link by embedding the entire transmitter unit (joysticks, MCU, and radio) into a single, cohesive device.
