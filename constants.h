// Constants used by bicycle computer

// Density of air
const float rho = 1.293;

// CdA figure for bike and rider.
// From https://ridefar.info/bike/cycling-speed/air-resistance-cyclist/
//
// Aerobars	0.37
// Drops	0.4
// Hoods	0.42
// Tops		0.46
// Standing	0.5
const float cda = 0.42;

// Mass of bike and rider conbined
const float mass = 90;

// Rolling resistance coefficient
const float crr = 0.007;

// Circumference of wheel used to measure speed
const float circ = 2.30;

// Derived constants
const float crrmg = crr * mass * 9.8;
const float half_acd_rho = 0.5 * cda * rho;

// Define this if there is a cadence sensor
#undef CADENCE_SUPPORTED

// Pins for wheel and cadence sensors
#define WHEEL_PIN 10
#define CRANK_PIN 9

// Output pin for driving motor's wheel sensor input
// (high pulse pulls OC output low)
#define OUTPUT_PIN 8

// Pend_yz assumes 33BLE is positioned with long axis across the bike.
// Define this if the USB port points to the left.
#undef USB_POINTS_LEFT

// Reporting/averaging interval in ms
#define REPORTING_INTERVAL    500

// Interval (in ms) after which sensors are considered inactive
#define INACTIVITY_INTERVAL   4000

// Median filter stores FILTER_SIZE elements and (optionally) averages the middle FILTER_WINDOW of them.
// There should be roughly FILTER_SIZE readings taken in a period of one or more REPORTING_INTERVAL's.
// (at the IMU default of ~110 Hz)
// FILTER_SIZE should ideally be odd to stop sampling bias.
#define FILTER_SIZE     129
#define FILTER_WINDOW   10

// Multiplier cutoff. 
// Below this speed the multiplier is 1.0
// When speed threshold passed, multiplier kicks in
// When slowing down, multiplier remains until we are below (threshold-offset)
#define MULTIPLIER      1.18f
#define SPEED_THRESHOLD 24
#define SPEED_OFFSET    4

// The size of the output pulse, expressed as a fraction of the wheel revolution.
// (the same as the magnet size as a fraction of the path it takes around the wheel)
// This is given as a denominator, e.g. 144 means magnet is 1/144 of the wheel rev
#define OUTPUT_PULSE_FRAC 144


