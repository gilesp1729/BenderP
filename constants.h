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

// Pend_yz assumes 33BLE is positioned with long axis across the bike.
// Define this if the USB port points to the left.
#undef USB_POINTS_LEFT

// Reporting/averaging interval in ms
#define REPORTING_INTERVAL    800

// Interval (in ms) after which sensors are considered inactive
#define INACTIVITY_INTERVAL   4000



