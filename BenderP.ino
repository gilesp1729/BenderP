#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <RunningMedian.h>
#include <NanoBLEFlashPrefs.h>

// Cycle computer with cadence, speed and opposing-force power meter.

#include "constants.h"

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
float pend_xy, pend_yz, pend_zx;
float force_input;
int connected = 0;

#ifdef CADENCE_SUPPORTED
// Feature bits: bit 2 - wheel pair present, bit 3 - crank pair present
unsigned short feature_bits = 0x0C;

// Flags bits: bit 4 - wheel pair present, bit 5 - crank pair present
unsigned short flags = 0x30;

#else // no crank bits
unsigned short feature_bits = 0x04;
unsigned short flags = 0x10;
#endif

// Median filter
RunningMedian filter(FILTER_SIZE);

// Access to the flash (non-volatile) memory
NanoBLEFlashPrefs flash;

// Struct to be stored in flash. Holds calibration data.
typedef struct Config
{
  float xc, yc, zc;     // Static config values (accelerometer readings)
  bool is_valid;        // Set to true if config is valid and can be used on power-up
} Config;

Config config = {0, 0, 0, 0};

// Timing and counters
volatile unsigned long previousMillis = 0;
volatile unsigned long currentMillis = 0;

// Counters for wheel and crank interrupt routines
volatile unsigned long time_prev_wheel = 0, time_now_wheel;
volatile unsigned long time_prev_crank = 0, time_now_crank;
volatile unsigned long time_chat = 100;  // dead zone for bounces
unsigned long time_flash_slope = 0;
int flash_slope = 0;

// Counters for updating power and speed services
volatile unsigned long wheelRev = 0;
volatile unsigned long oldWheelRev = 0;
volatile unsigned long oldWheelMillis = 0;  // last time sent to BLE
volatile unsigned long lastWheeltime = 0;   // last time measurement taken
// Note: the wheel time is in half-ms (1/2048 sec), unlike CSC where it is in ms
volatile float speed = 0;                   // calculated speed in metres/sec

volatile unsigned int crankRev = 0;
volatile unsigned int oldCrankRev = 0;
volatile unsigned long lastCranktime = 0;
volatile unsigned long oldCrankMillis = 0;

volatile unsigned int outputWheelRev = 0;
volatile unsigned long outputWheeltime = 0;
volatile unsigned int outputInterval = 0;


// Fill the CP measurement array and send it
void fillCP() 
{
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


// Update old values and send CP and CSC to BLE client
void update_chars(bool calc_power, String sType) 
{

  if (calc_power && filter.getCount() > 0) 
  {
    // Collect the pend angle from the last update interval using a median cut.
    pend_yz = filter.getMedian();
    //pend_yz = filter.getMedianAverage(FILTER_WINDOW);

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
  Serial.print(" (");
  Serial.print(filter.getLowest());
  Serial.print(" to ");
  Serial.print(filter.getHighest());
  Serial.print(") Speed : ");
  Serial.print(speed);
  Serial.print(" Power : ");
  Serial.print(power);
  Serial.print("  ");
  Serial.println(sType);

  Serial.print("Output Wheel Rev.: ");
  Serial.print(outputWheelRev);
  Serial.print(" OutputWheelTime : ");
  Serial.println(outputWheeltime);
}

// Interrupt routines trigger when a pulse is received (falling edge on pin)
void wheelAdd() 
{
  time_now_wheel = millis();
  if (time_now_wheel > time_prev_wheel + time_chat) 
  {
    // Calculate the speed in m/s based on wheel circumference (needed for power calcs)
    speed = (circ * 1000) / (time_now_wheel - time_prev_wheel);

    // Calculate the output speed by interpolation (TODO. For now just approximate)
    float output_speed = speed * 25.0 / 32.0;

    // And the output interval to use for the next output pulse
    outputInterval = (circ * 1000) / output_speed;

    // Update the wheel counter and remember the time of last update
    wheelRev = wheelRev + 1;
    time_prev_wheel = time_now_wheel;
    lastWheeltime = millis() << 1;

    // If we are not connected to a central, we are just echoing out the
    // wheel sensor pulses to the output.
    if (!connected)
    {
      // Output a pulse to the motor input pin
      digitalWrite(OUTPUT_PIN, HIGH);
      delay(3);   // TODO: Can/should I be doing this in an interrupt routine?
      digitalWrite(OUTPUT_PIN, LOW);
    }
  }
}

void crankAdd() 
{
  time_now_crank = millis();
  if (time_now_crank > time_prev_crank + time_chat) {
    crankRev = crankRev + 1;
    time_prev_crank = time_now_crank;
    lastCranktime = millis();
  }
}

// Short routine to print XYZ values from accelerometer.
void println_xyz(float x, float y, float z) 
{
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z);
}

void setup() 
{
  int count = 0;

  Serial.begin(9600);  // initialize serial communication
  while (!Serial) 
  {  
    // Be sure to break out so we don't wait forever if no serial is connected
    if (count++ > 20)
      break;
    delay(100);
  }
 
  if (!IMU.begin()) 
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  // Wait for an acceleration figure to settle.
  Serial.print("Checking calibration: ");
  delay(200);
  while (!IMU.accelerationAvailable());
  IMU.readAcceleration(x, y, z);
  println_xyz(x, y, z);

  if (fabs(x) < 0.05 && fabs(y) < 0.05 && z > 0.93) {
    // Device is lying flat withe Z-axis pointing up. 
    // (loose limits to cope with variations in boards)
    // Store the calibration but don't mark it valid.
    // The next power-up when mounted on the bike will calibrate again and store it.
    Serial.println("Device is flat - resetting calibration");
    config.xc = x;
    config.yc = y;
    config.zc = z;
    config.is_valid = 0;
    flash.deletePrefs();
    flash.garbageCollection();
    flash.writePrefs(&config, sizeof(config));
  }
  else
  {
    // We're not lying flat. Read the config from flash
    flash.readPrefs(&config, sizeof(config));
    if (config.is_valid) 
    {
      // The config is valid. Just use it.
      Serial.print("Using valid configuration: ");
      println_xyz(config.xc, config.yc, config.zc);      
    }
    else
    {
      // Not valid, so write it and mark as valid for next power-up
      Serial.println("Device is not flat - storing calibration: ");
      config.xc = x;
      config.yc = y;
      config.zc = z;
      config.is_valid = 1;
      println_xyz(config.xc, config.yc, config.zc);      
      flash.writePrefs(&config, sizeof(config));
    }
  }
  Serial.println(flash.statusString());
  
  // Initialize the LED on pin 13 to indicate when a central is connected
  pinMode(LED_BUILTIN, OUTPUT);  
  digitalWrite(LED_BUILTIN, LOW);

  // Initialise BLE and establish cycling power characteristics
  BLE.begin();
  BLE.setLocalName("Bender Power");
  BLE.setAdvertisedService(CyclePowerService);
  CyclePowerService.addCharacteristic(CyclePowerFeature);
  CyclePowerService.addCharacteristic(CyclePowerMeasurement);
  CyclePowerService.addCharacteristic(CyclePowerSensorLocation);
  BLE.addService(CyclePowerService);

  // Don't advertise this service; it will be found when the app connects,
  // if the app is looking for it
  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);
  batteryLevelChar.writeValue(oldBatteryLevel);

  lastWheeltime = millis() << 1;
  lastCranktime = millis();

  unsigned long t = millis();
  lastWheeltime = t << 1;       // this is in half-ms
  lastCranktime = t;
  outputWheeltime = t;
  
  // Write the initial values of the CP (power) characteristics
  slBuffer[0] = sensor_pos & 0xff;
  fBuffer[0] = feature_bits & 0xff;   // little endian
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x00;
  CyclePowerFeature.writeValue(fBuffer, 4);
  CyclePowerSensorLocation.writeValue(slBuffer, 1);
  fillCP();

  // Set up output pin
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);

  // Attach the wheel and crank interrupt routines to their input pins
  attachInterrupt(digitalPinToInterrupt(WHEEL_PIN), wheelAdd, FALLING);
#ifdef CADENCE_SUPPORTED
  attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crankAdd, FALLING);
#endif

  // Advertise that we are ready to go
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() 
{
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) 
  {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) 
    {
      connected = 1;
      currentMillis = millis();

      // Calculate the pendulum angle from the accelerometer
      // and hence find the current power consumption.
      // Every time it's available, accumulate the pend angle
      if (IMU.accelerationAvailable()) 
      {
        float yz;

        IMU.readAcceleration(x, y, z);

       // Pend_yz assumes 33BLE is positioned with long axis across
       // the bike, and the USB port points to the right.
        // pend_xy = y * config.xc - x * config.yc;
        // pend_zx = z * config.xc - x * config.zc;
#ifdef USB_POINTS_LEFT        
        yz = z * config.yc - y * config.zc;
#else 
        yz = y * config.zc - z * config.yc;
#endif
        filter.add(yz);
      }

      // If there an output interval, count it down till it expires. Then update the
      // output wheel counters
      if (outputInterval > 0 && currentMillis - outputWheeltime >= outputInterval)
      {
        outputWheelRev++;
        outputWheeltime = currentMillis;

        // Output a pulse to the motor input pin
        digitalWrite(OUTPUT_PIN, HIGH);
        delay(3);
        digitalWrite(OUTPUT_PIN, LOW);
      }

      // Check and report the wheel and crank measurements every REPORTING_INTERVAL ms
      if (oldWheelRev < wheelRev && currentMillis - oldWheelMillis >= REPORTING_INTERVAL) 
      {
        update_chars(1, "wheel");
      } 
      else if (oldCrankRev < crankRev && currentMillis - oldCrankMillis >= REPORTING_INTERVAL) 
      {
        update_chars(1, "crank");
      } 
      else if (currentMillis - previousMillis >= REPORTING_INTERVAL) 
      {
        // simulate some speed on the wheel, 500ms per rev ~16km/h, 800ms ~10km/h
      // wheelAdd();
      // crankAdd();
        update_chars(0, "timer");
      }

      // if the wheel timer has not been updated for 4 seconds, zero out the
      // internal speed value, so the power resets to zero.
      if (currentMillis > time_prev_wheel + INACTIVITY_INTERVAL)
      {
        speed = 0;
        outputInterval = 0;
      }

      // If speed is zero, flash out the current percent slope. This is a debug
      // output on the pend_angle and verifies a correct static calibration.
      // NOTE: SuperCycle must be set to not do any smoothing on power.
      if (speed < 0.001)
      {
        if (currentMillis - time_flash_slope > 1000)
        {
          if (flash_slope)
          {
            pend_yz = filter.getMedian();
            power = pend_yz * 100;        // TODO convert sin to tan
            Serial.print("Flashing a slope of ");
            Serial.print(power);
            Serial.println(" percent");
          }
          else
          {
            power = 0;
          }
          fillCP();
          flash_slope ^= 1;
          time_flash_slope = currentMillis;
        }
      }
      else
      {
        time_flash_slope = currentMillis;
      }
    }

    // when the central disconnects, turn off the LED:
    connected = 0;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");
  }
}
