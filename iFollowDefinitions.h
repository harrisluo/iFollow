// Blynk Auth
char auth[] = "3cIMzOvMabyrD2jCGUfv_q-MrHU-ydPM";

// Pin variables

#define GPS_TX_PIN 1

#define BLUETOOTH_TX_PIN 10
#define BLUETOOTH_RX_PIN 11

#define MOTOR_A_EN_PIN 5
#define MOTOR_B_EN_PIN 3
#define MOTOR_A_IN_1_PIN 6
#define MOTOR_A_IN_2_PIN 7
#define MOTOR_B_IN_1_PIN 2
#define MOTOR_B_IN_2_PIN 4

// If one motor tends to spin faster than the other, add offset
#define MOTOR_A_OFFSET 0
#define MOTOR_B_OFFSET 0


// How often the GPS should update in MS
#define GPS_UPDATE_INTERVAL 1000

// Number of changes in movement to timeout for GPS streaming
// Keeps the cooler from driving away if there is a problem
#define GPS_STREAM_TIMEOUT 18

// Number of changes in movement to timeout for GPS waypoints
// Keeps the cooler from driving away if there is a problem
#define GPS_WAYPOINT_TIMEOUT 45

struct GeoLoc {
  float lat;
  float lon;
};
