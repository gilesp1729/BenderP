#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// Cycle computer with cadence, speed and opposing-force power meter.

#include "constants.h"

// Bluetooth® Low Energy Cycling Power service (CP)
BLEService cscService("1816");

// Service and characteristics
BLEService CyclePowerService("1818");
BLECharacteristic CyclePowerFeature("2A65", BLERead, 4);
BLECharacteristic CyclePowerMeasurement("2A63", BLERead | BLENotify, 14);
BLECharacteristic CyclePowerSensorLocation("2A5D", BLERead, 1);

// Battery service, in case it is needed. Even if not, it keeps
// the display tidy on apps that display it
BLEService batteryService("180F");

// Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);
int oldBatteryLevel = 100;  // start with 100% full

int sensor_pos = 11;      // sensor position magic number. 
// No idea what they mean (shitty specs) but it's mandatory to supply one.

unsigned char bleBuffer[14];
unsigned char slBuffer[1];
unsigned char fBuffer[4];
short power = 0;
float x, y, z;
float xc, yc, zc;
float pend_xy, pend_yz, pend_zx;
float force_input, speed;
int pend_count = 0;

#ifdef CADENCE_SUPPORTED
// Feature bits: bit 2 - wheel pair present, bit 3 - crank pair present
unsigned short feature_bits = 0x0C;

// Flags bits: bit 4 - wheel pair present, bit 5 - crank pair present
unsigned short flags = 0x30;

#else // no crank bits
unsigned short feature_bits = 0x04;
unsigned short flags = 0x10;
#endif

// Pins for wheel and cadence sensors
#define WHEEL_PIN 10
#define CRANK_PIN 9

// Analog pin for the current sensor
#define CURRENT_PIN A0

// Timing and counters
volatile unsigned long previousMillis = 0;
volatile unsigned long currentMillis = 0;

volatile unsigned long time_prev_wheel = 0, time_now_wheel;
volatile unsigned long time_prev_crank = 0, time_now_crank;
volatile unsigned long time_chat = 100;  // dead zone for bounces

volatile unsigned long wheelRev = 0;
volatile unsigned long oldWheelRev = 0;
volatile unsigned long oldWheelMillis = 0;  // last time sent to BLE
volatile unsigned long lastWheeltime = 0;   // last time measurement taken
// Note: the wheel time is in half-ms (1/2048 sec), unlike CSC where it is in ms

volatile unsigned int crankRev = 0;
volatile unsigned int oldCrankRev = 0;
volatile unsigned long lastCranktime = 0;
volatile unsigned long oldCrankMillis = 0;


// Fill the CP measurement array and send it
void fillCP() {
  int n = 0;  // to facilitate adding and removing stuff
  bleBuffer[n++] = flags & 0xff;
  bleBuffer[n++] = (flags >> 8) & 0xff;
  bleBuffer[n++] = power & 0xff;
  bleBuffer[n++] = (power >> 8) & 0xff;
  bleBuffer[n++] = wheelRev & 0xff;
  bleBuffer[n++] = (wheelRev >> 8) & 0xff;	// UInt32
  bleBuffer[n++] = (wheelRev >> 16) & 0xff;
  bleBuffer[n++] = (wheelRev >> 24) & 0xff;
  bleBuffer[n++] = lastWheeltime & 0xff;
  bleBuffer[n++] = (lastWheeltime >> 8) & 0xff;
#ifdef CADENCE_SUPPORTED
  bleBuffer[n++] = crankRev & 0xff;
  bleBuffer[n++] = (crankRev >> 8) & 0xff;
  bleBuffer[n++] = lastCranktime & 0xff;
  bleBuffer[n++] = (lastCranktime >> 8) & 0xff;
#endif

  CyclePowerMeasurement.writeValue(bleBuffer, n);
}

// Update old values and send CP
void updateCP(String sType) {

  if (pend_count > 0) {
    // Average the pend angles collected in the last update interval
    pend_yz /= pend_count;

    // calculate the power usage by adding the forces and multiplying by speed
    // force input = force total (from accelerometer) + air and rolling drag forces
    // then power = force input * speed
    force_input = (mass * 9.8 * pend_yz) + (half_acd_rho * speed * speed) + crrmg;
    power = force_input * speed;
  }

  // Update old values and send CP
  oldWheelRev = wheelRev;
  oldCrankRev = crankRev;
  oldWheelMillis = currentMillis;
  previousMillis = currentMillis;
  fillCP();

  // Some debug output to indicate what triggered the update
  Serial.print("Wheel Rev.: ");
  Serial.print(wheelRev);
  Serial.print(" WheelTime : ");
  Serial.print(lastWheeltime);
#ifdef CADENCE_SUPPORTED
  Serial.print(" Crank Rev.: ");
  Serial.print(crankRev);
  Serial.print(" CrankTime : ");
  Serial.print(lastCranktime);
#endif
  Serial.print(" Pend angle : ");
  Serial.print(pend_yz);
  Serial.print(" # of readings : ");
  Serial.print(pend_count);
  Serial.print(" Speed : ");
  Serial.print(speed);
  Serial.print(" Power : ");
  Serial.print(power);
  Serial.print("  ");
  Serial.println(sType);

  // Zero the counters
  pend_count = 0;
  pend_yz = 0;
}

void setup() {
  Serial.begin(9600);  // initialize serial communication
  /*
  while (!Serial) {  / DO NOT DO THIS if no serial is connected. It waits forever
    ;  // Wait for ready
  }
  */
  delay(500);  // wait for serial, but not forever
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  // Wait for an acceleration figure to settle and record the baseline.
  // For now, do this on every startup. Eventually, it will be done once
  // and written to non-volatile memory.
  Serial.println("Calibrating");
  delay(100);
  while (!IMU.accelerationAvailable());
  IMU.readAcceleration(xc, yc, zc);
  Serial.print(xc);
  Serial.print(" ");
  Serial.print(yc);
  Serial.print(" ");
  Serial.println(zc);
  
  pinMode(LED_BUILTIN, OUTPUT);  // initialize the LED on pin 13 to indicate when a central is connected
  digitalWrite(LED_BUILTIN, LOW);

  // Initialise BLE and establish cycling power characteristics
  BLE.begin();
  BLE.setLocalName("Bender Power");
  BLE.setAdvertisedService(CyclePowerService);
  CyclePowerService.addCharacteristic(CyclePowerFeature);
  CyclePowerService.addCharacteristic(CyclePowerMeasurement);
  CyclePowerService.addCharacteristic(CyclePowerSensorLocation);
  BLE.addService(CyclePowerService);

  // Don't advertise this service; it will be found when the app connects
  // if the app is looking for it
  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);
  batteryLevelChar.writeValue(oldBatteryLevel);

  lastWheeltime = millis() << 1;
  lastCranktime = millis();

  // Write the initial values of all the characteristics

  slBuffer[0] = sensor_pos & 0xff;
  fBuffer[0] = feature_bits & 0xff;   // little endian
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x00;
  CyclePowerFeature.writeValue(fBuffer, 4);
  CyclePowerSensorLocation.writeValue(slBuffer, 1);

  fillCP();

  attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), wheelAdd, FALLING);
#ifdef CADENCE_SUPPORTED
  attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crankAdd, FALLING);
#endif

  // Advertise that we are ready to go
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

// Interrupt routines trigger when a pulse is received (falling edge on pin)
void wheelAdd() {
  time_now_wheel = millis();
  if (time_now_wheel > time_prev_wheel + time_chat) {
    // Calculate the speed in m/s based on wheel circumference (needed for power calcs)
    speed = (circ * 1000) / (time_now_wheel - time_prev_wheel);
    wheelRev = wheelRev + 1;
    time_prev_wheel = time_now_wheel;
    lastWheeltime = millis() << 1;
  }
}

void crankAdd() {
  time_now_crank = millis();
  if (time_now_crank > time_prev_crank + time_chat) {
    crankRev = crankRev + 1;
    time_prev_crank = time_now_crank;
    lastCranktime = millis();
  }
}

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      currentMillis = millis();

      // Calculate the pendulum angle from the accelerometer
      // and hence find the current power consumption.
      // Every time it's available, accumulate and average the pend angle
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);

       // Pend_yz assumes 33BLE is positioned with long axis across
       // the bike, and the USB port points to the right.
        // pend_xy = y * xc - x * yc;
        // pend_zx = z * xc - x * zc;
#ifdef USB_POINTS_LEFT        
        pend_yz += z * yc - y * zc;
#else 
        pend_yz += y * zc - z * yc;
#endif
        pend_count++;
      }

      // check the csc measurement every 500ms
      if (oldWheelRev < wheelRev && currentMillis - oldWheelMillis >= 500) {
        updateCP("wheel");
      } else if (oldCrankRev < crankRev && currentMillis - oldCrankMillis >= 500) {
        updateCP("crank");
      } else if (currentMillis - previousMillis >= 500) {
        // simulate some speed on the wheel, 500ms per rev or ~16km/h
      // wheelAdd();
      // crankAdd();
        updateCP("timer");
      }
    }

    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");
  }
}
