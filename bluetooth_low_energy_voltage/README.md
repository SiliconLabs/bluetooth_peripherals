# BLE Low Energy Voltage Measurement #

## Summary ##

This example incorporates low energy peripherals such as LETimer, ADC, PRS, and LDMA with the Bluetooth stack which notifies clients of measured voltage data.

## Gecko SDK version ##

v2.7

## Hardware Required ##

- One BRD4162A EFR32MG12 radio board
- One BRD4001A WSTK board

## Setup ##

Import the included .sls project into Studio, build, and flash the project onto the device. Open a COM port to the device using a program like TeraTerm to see the device's address. Connect to the device and read the ADC characteristic to get the measured voltage value

## How It Works ##

This project configures multiple peripherals to work together in low power EM2 mode. The application is capable of triggering the ADC Conversion from a GPIO button press or a LETimer timeout through PRS while in EM2. Once the ADC conversion is completed, the system transitions to EM1 and the LDMA copies the data into a buffer. Since the LDMA triggers an interrupt whenever the memory buffer is full, the interrupt handler calls gecko_external_signal() to create an event so the main thread can handle the notification.

## .sls Projects Used ##

- bluetooth_low_energy_voltage_mg12.sls

## How to Port to Another Part ##

Open the "Project Properties" and navigate to the "C/C++ Build -> Board/Part/SDK" item. Select the new board or part to target and "Apply" the changes. Note: there may be dependencies that need to be resolved when changing the target architecture.
