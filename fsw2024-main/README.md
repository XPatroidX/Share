# fsw2024
CanSat 2024 Flight Software

## Project Description
This repository contains the flight software developed by the Sapienza Space Team for the 2024 Cansat competition. The software is designed to operate on a Raspberry Pi Pico 2040 development board. It handles the payload's data acquisition, processing, and transmission during the flight, ensuring reliable operation under the competition's requirements.

## Flight Software Requirements
1. **Installed Zephyr SDK**
2. **Cloned Zephyr with its modules and installed Python libraries**
3. **Exported `ZEPHYR_BASE` environment variable**

A getting started guide is available [here](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

## Building and Running Commands

### Building the Project
To build the project on a Raspberry Pi Pico 2040 board when inside this project folder:
```
west build -b rpi_pico
```
### Running the Software
1. Connect the Raspberry Pi Pico 2040 to your computer via USB while holding down the BOOTSEL button
2. A new drive will appear on your computer
3. Copy the uf2 file to this new drive
4. The Raspberry Pi Pico 2040 will automatically reboot and run the new software


