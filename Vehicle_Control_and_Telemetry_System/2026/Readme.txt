================================================================
MAVERA TECHNOLOGY TEAM - VEHICLE CONTROL AND TELEMETRY SYSTEM (2026)
================================================================

This repository contains the embedded software, hardware design, and system diagrams developed for the electric vehicle's Vehicle Control and Telemetry System, as part of the Mavera Technology Team.


----------------------------------------------------------------
PROJECT OVERVIEW
----------------------------------------------------------------

The system runs on an STM32F4 microcontroller and handles battery monitoring, insulation resistance measurement, motor speed/torque calculation, HMI display communication, SD card logging, and remote telemetry transmission for the team's electric vehicle.


----------------------------------------------------------------
FEATURES
----------------------------------------------------------------

- BMS communication over CAN Bus (cell voltages, temperatures, current) and charge MOSFET control
- Insulation resistance monitoring via Modbus RTU (Inotel IMD100), with fault-resistance detection and calibration
- Hall-effect based speed/RPM calculation (Timer Input Capture), with real-time torque and mechanical power calculation
- Nextion HMI display integration (voltage, temperature, speed, torque, warnings)
- SD card data logging (FATFS), timestamped CSV format
- GSM-based remote telemetry, JSON data sent over HTTP
- Threshold-based safety outputs (cooling / relay control) for temperature and insulation faults


----------------------------------------------------------------
HARDWARE
----------------------------------------------------------------

- STM32F4 series microcontroller
- Nextion HMI touchscreen
- Daly BMS
- Inotel IMD100 insulation monitor
- GSM module
- Hall effect speed sensor
- SD card module
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

MAVERA
|
|-- Software/
|   `-- Main Code
|
|-- Hardware/
|   |-- PCB_2D
|   `-- Schematic
|
|-- Diagrams/
|
`-- README.txt


----------------------------------------------------------------
DEVELOPMENT PROCESS
----------------------------------------------------------------

- CAN Bus communication with the BMS
- Modbus RTU communication with the insulation monitor
- Hall sensor based speed/torque calculation
- Nextion HMI communication
- SD card data logging with FATFS
- GSM/HTTP telemetry transmission
- Safety threshold logic and system integration testing


----------------------------------------------------------------
TECHNOLOGIES
----------------------------------------------------------------

- Microcontroller: STM32F4
- Programming: C
- IDE: STM32CubeIDE
- Embedded Framework: STM32 HAL
- Communication: CAN Bus, Modbus RTU, UART, SPI
- Data Logging: FATFS / SD Card
- HMI: Nextion Display
- Wireless Telemetry: GSM / HTTP
- Hardware Design: PCB / Schematic Design


----------------------------------------------------------------
AUTHOR
----------------------------------------------------------------

Atakan Akdogan
Electrical & Electronics Engineering Student
GitHub: https://github.com/atakannakdogan-boop