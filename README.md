# BenderP
Bike power meter with speed and cadence. Runs on Arduino 33 BLE or BLE Sense.

Opposing-force power meter with BLE connection to popular cycling apps. Only sensors required are a Hall sensor with a wheel magnet for speed,
and optionally another one for cadence.

Libraries used:

- ArduinoBLE - Bluetooth Low Energy
- Arduino_LSM9DS1 - Arduino 33 BLE series IMU
- RunningMedian - median filtering library used to clean up accelerometer noise
- NanoBLEFlashPrefs - Non-volatile storage library for 33 BLE series

Connections:

A Bosch Hall sensor works well from 3.3V, and can be mounted alonside the existing one to be triggered by the wheel magnet.
Connect positive (red) wire to Pin 10 with a 3.3k pullup to +3.3V, negative (black) to ground. If using a cadence sensor
do the same with Pin 9.

12V can be had from the Bosch Gen 4 motors using the freely available 3rd-party cable, or from earlier motors by splicing 
into one of the light cables.

Calibration

To reset the calibration, power up or reset the board when lying on a flat level surface. When mounted on the bike,
the accelerometer orientation is stored in non-volatile memory on the next power up and used thereafter.


Works in progress:

- Dynamic calibration of air and rolling resistance coefficients when not pedalling (requires cadence sensor)
- Speed bending to control the speed limit on ebike motors depending on a wheel magnet sensor.

