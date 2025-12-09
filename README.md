# EEE 192 PIR Sensor Node

**Author:** Christian Nikolai Rabaya  
**Date:** June 12, 2024

## Project Overview

This project, created for the EEE 192 course, implements a standalone sensor node using an STM32F411RE microcontroller. The system reads data from three Passive Infrared (PIR) sensors and uploads their status to a ThingSpeak channel using an ESP8266 Wi-Fi module. It serves as a practical example of an IoT device for remote monitoring.

The application is built using the STM32 HAL libraries and is developed within the STM32CubeIDE environment. It is heavily inspired by previous coursework from EEE 158.

## Features

-   **Multi-Sensor Input:** Reads digital states from three separate PIR sensors.
-   **Wi-Fi Connectivity:** Utilizes an ESP8266 module (controlled via AT commands) to connect to a Wi-Fi network.
-   **Cloud Data Logging:** Pushes sensor data (1 for motion detected, 0 for none) to three distinct fields on a ThingSpeak channel.
-   **Visual Feedback:** An on-board LED provides a visual indication when any of the PIR sensors detect motion.
-   **System Reliability:** An Independent Watchdog (IWDG) is implemented to automatically reset the microcontroller if the main loop hangs, ensuring continuous operation.
-   **Debug Interface:** A secondary USART interface is configured for sending debug messages.

## Hardware Requirements

-   STM32F411RE Nucleo-64 Board
-   ESP8266 Wi-Fi Module
-   3x PIR Motion Sensors
-   Status LED (or use the on-board LED)
-   Connecting wires

## Pin Configuration

The following GPIO pins are used on the STM32 microcontroller:

| Pin   | Function                  | Connection                |
| :---- | :------------------------ | :------------------------ |
| `PA0` | Digital Input             | PIR Sensor 1 Output       |
| `PA1` | Digital Input             | PIR Sensor 2 Output       |
| `PA4` | Digital Input             | PIR Sensor 3 Output       |
| `PA6` | GPIO Output               | Status LED (Active High)  |
| `PA9` | `USART1_TX`               | ESP8266 RX Pin            |
| `PA10`| `USART1_RX`               | ESP8266 TX Pin            |
| `PA2` | `USART2_TX` (Debug)       | Serial-to-USB RX          |
| `PA3` | `USART2_RX` (Debug)       | Serial-to-USB TX          |
| `PC13`| On-board User Pushbutton  | (Configured but not used in main loop) |


## How to Use

### 1. Configuration

Before flashing the firmware, you must configure the Wi-Fi and ThingSpeak credentials.

1.  Open the project in STM32CubeIDE.
2.  Navigate to `Core/Src/main.c`.
3.  Locate the `ESP_Init()` function call inside `main()` and replace the placeholder credentials with your Wi-Fi network's **SSID** and **Password**.
    ```c
    // Original
    ESP_Init("GlobeAtHome_c9920_2.4", "kingkongMASTER9413");

    // Example replacement
    ESP_Init("MyWiFi_SSID", "MyWiFi_Password");
    ```
4.  In the `ESP_Send()` function, find the `API_KEY` constant and replace it with your ThingSpeak channel's **Write API Key**.
    ```c
    const char *API_KEY = "YOUR_THINGSPEAK_API_KEY";
    ```

### 2. Build and Flash

1.  Connect the Nucleo board to your computer via USB.
2.  Build the project in STM32CubeIDE (Project -> Build All).
3.  Flash the firmware to the board (Run -> Debug As -> STM32 MCU C/C++ Application).

### 3. Hardware Connection

Connect the external components (PIR sensors, ESP8266) to the Nucleo board according to the [Pin Configuration](#pin-configuration) table.

### 4. Operation

Once powered on, the device will automatically connect to the configured Wi-Fi network and begin sending sensor data to ThingSpeak every ~9 seconds. The status LED on `PA6` will light up for 1.5 seconds whenever motion is detected by any of the sensors. Debug messages are transmitted over `USART2` at a baud rate of 115200.
