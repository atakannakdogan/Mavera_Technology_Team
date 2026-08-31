================================================================
MAVERA TECHNOLOGY TEAM - VEHICLE CONTROL AND TELEMETRY SYSTEM (2024)
================================================================

This repository contains the embedded software, PCB design, system architecture, and testing work developed for the electric vehicle's Vehicle Control and Telemetry System, as part of the Mavera Technology Team's 2024 season.


----------------------------------------------------------------
PROJECT OVERVIEW
----------------------------------------------------------------

During the 2024 season, development focused on sensor data acquisition and processing, CAN Bus communication, data logging, GPS tracking, vehicle display integration, and telemetry data transmission. The system runs on an STM32 microcontroller with custom embedded firmware.


----------------------------------------------------------------
FEATURES
----------------------------------------------------------------

- Analog sensor data acquisition via ADC (voltage, current, temperature)
- CAN Bus communication with vehicle subsystems (cell voltages, battery temperatures, isolation resistance)
- RMS-based current measurement with digital filtering
- GPS data acquisition via UART (speed, satellites in use)
- SD card data logging (FATFS), with a new log file created per session
- Nextion HMI display integration (real-time voltage, temperature, speed, energy, GPS, warnings)
- UART-based telemetry data transmission


----------------------------------------------------------------
HARDWARE
----------------------------------------------------------------

- STM32 microcontroller
- Nextion HMI display
- GPS module (UART / NMEA)
- SD card module
- CAN Bus transceiver
- UART-based telemetry module
- Custom PCB


----------------------------------------------------------------
SOFTWARE & TOOLS
----------------------------------------------------------------

- C
- STM32CubeIDE
- STM32 HAL
- FATFS


----------------------------------------------------------------
PROJECT STRUCTURE
----------------------------------------------------------------

MAVERA-2024
|
|-- Software/
|   `-- Main Code
|
|-- Hardware/
|   |-- PCB_2D
|   `-- Schematic
|
`-- Diagrams/


----------------------------------------------------------------
DEVELOPMENT PROCESS
----------------------------------------------------------------

- CAN Bus communication and BMS data parsing
- ADC-based sensor acquisition and filtering
- GPS (NMEA) integration
- SD card data logging with FATFS
- Nextion HMI communication
- Telemetry data framing and transmission
- Individual and integrated system testing


----------------------------------------------------------------
TESTING & VALIDATION
----------------------------------------------------------------

Hardware and software components were tested and debugged individually and as part of the integrated system, covering CAN Bus, UART, ADC, GPS, SD card, sensor acquisition, data logging, and Nextion display communication.


----------------------------------------------------------------
TECHNOLOGIES
----------------------------------------------------------------

- Microcontroller: STM32
- Programming: C
- IDE: STM32CubeIDE
- Embedded Framework: STM32 HAL
- Communication: CAN Bus, UART, SPI
- Data Acquisition: ADC
- GPS: UART / NMEA
- Data Logging: FATFS / SD Card
- HMI: Nextion Display
- Wireless Telemetry: UART-based module
- Hardware Design: PCB / Schematic Design


----------------------------------------------------------------
MY ROLE
----------------------------------------------------------------

This repository represents my 2024 work on the Vehicle Control and Telemetry System within the Mavera Technology Team. I worked on STM32-based embedded software development, CAN Bus communication, sensor integration, GPS, SD card data logging, Nextion HMI, telemetry, and PCB design.


----------------------------------------------------------------
AUTHOR
----------------------------------------------------------------

Atakan Akdogan
Electrical & Electronics Engineering Student
GitHub: https://github.com/atakannakdogan-boop