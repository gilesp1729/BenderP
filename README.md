# BenderP
Bike power meter with speed, cadence and dynamic calibration. Runs on Arduino 33 BLE or BLE Sense.

Opposing-force power meter with BLE connection to popular cycling apps. Only sensors required are a Hall sensor with a wheel magnet.

Libraries used:

ArduinoBLE - Bluetooth Low Energy
Arduino_LSM9DS1 - Arduino 33 BLE series IMU
RunningMedian - median filtering library used to clean up accelerometer noise
NanoBLEFlashPrefs - Non-volatile storage library for 33 BLE series (coming soon)

Works in progress:

Static calibration of accelerometer orientation stored in non-volatile memory when first power up on a flat level surface.
Dynamic calibration of air and rolling resistance coefficients when not pedalling (requires cadence sensor)

