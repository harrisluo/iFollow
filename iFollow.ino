#define BLYNK_USE_DIRECT_CONNECT

// Imports
#include <TinyGPS++.h>
#include <Wire.h>
//#include <Adafruit_HMC5883_U.h>

//NEW COMPASS
#include <HMC5883L_Simple.h>

#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include "./iFollowDefinitions.h"

// GPS
TinyGPSPlus gps;

//WidgetTerminal terminal(V3);

// Serial components
static const int RXPin = 0, TXPin = 1;
SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
SoftwareSerial ss(RXPin, TXPin);         // TXD to digital pin 6

/* Compass */
HMC5883L_Simple Compass;

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps); // comment
  }

  GeoLoc cartLoc;
  cartLoc.lat = 0.0;
  cartLoc.lon = 0.0;
  
  return cartLoc;
}

// Get and process GPS data
GeoLoc gpsdump(TinyGPSPlus &gps) {
/*  float flat, flon;
  unsigned long age;
  
  gps.f_get_position(&flat, &flon, &age);

  GeoLoc cartLoc;
  cartLoc.lat = flat;
  cartLoc.lon = flon;

  Serial.print(cartLoc.lat, 7); Serial.print(", "); Serial.println(cartLoc.lon, 7);

  return cartLoc;*/
    float flat, flon;
    flat = gps.location.lat();
    flon = gps.location.lng();
    GeoLoc cartLoc;
    cartLoc.lat = flat;
    cartLoc.lon = flon;
    Serial.print(cartLoc.lat, 7); Serial.print(", "); Serial.println(cartLoc.lon, 7);

    return cartLoc;

}

// Feed data as it becomes available 
bool feedgps() {
  
  while (ss.available()) {
    if (gps.encode(ss.read()))
    {
     
      return true;
    }
  }
  return false;
}

// GPS Streaming Hook
BLYNK_WRITE(V0) {
  GpsParam gps(param);
  
  Serial.println("Received remote GPS: ");
  
  // Print 7 decimal places for Lat
  Serial.print(gps.getLat(), 7); Serial.print(", "); Serial.println(gps.getLon(), 7);

  GeoLoc phoneLoc;
  phoneLoc.lat = gps.getLat();
  phoneLoc.lon = gps.getLon();

  driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
}

// Terminal Hook
/*BLYNK_WRITE(V7) {
  Serial.print("Received Text: ");
  Serial.println(param.asStr());

  String rawInput(param.asStr());
  int colonIndex;
  int commaIndex;
  
  do {
    commaIndex = rawInput.indexOf(',');
    colonIndex = rawInput.indexOf(':');
    
    if (commaIndex != -1) {
      String latStr = rawInput.substring(0, commaIndex);
      String lonStr = rawInput.substring(commaIndex+1);

      if (colonIndex != -1) {
         lonStr = rawInput.substring(commaIndex+1, colonIndex);
      }
    
      float lat = latStr.toFloat();
      float lon = lonStr.toFloat();
    
      if (lat != 0 && lon != 0) {
        GeoLoc waypoint;
        waypoint.lat = lat;
        waypoint.lon = lon;
    
        Serial.print("Waypoint found: "); Serial.print(lat); Serial.println(lon);
        driveTo(waypoint, GPS_WAYPOINT_TIMEOUT);
      }
    }
    
    rawInput = rawInput.substring(colonIndex + 1);
    
  } while (colonIndex != -1);
}

void displayCompassDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
*/

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  //Serial.print("f: ");
  return atan2(y, x) * RADTODEG;
}

float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float calibrate(float x) {
  float heading;
  if(x < 21) {
    heading = 0;
  } else if(x < 55.6) {
    heading = 1.81*x-38;
  } else if(x < 299.5) {
    heading = 0.24*x+48;
  } else {
    heading = 3*x-779;
  }
  if(heading < 0) heading += 360;
  if(heading >= 360) heading -= 360;

  return heading;
}

float geoHeading() {
  float heading = Compass.GetHeadingDegrees();
  heading = calibrate(heading);
  Serial.print("***geoHeading: ");
  Serial.println(heading);
  delay(1000);
  return heading;
}

void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_OFFSET);
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
  
  // set speed to 200 out of possible range 0~255
  analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_OFFSET);
}

void setSpeed(int speed)
{
  // this function will run the motors in both directions at a fixed speed
  // turn on motor A
  setSpeedMotorA(speed);

  // turn on motor B
  setSpeedMotorB(speed);
}

//

void stopDriving() {
  // now turn off motors
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}

//CHANGE 

void drive(int distance, float turn) {
  int fullSpeed = 230;
  int stopSpeed = 25;

  // drive to location
  int s = fullSpeed;
  /*if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
  }*/
  
  int autoThrottle = constrain(s, stopSpeed, fullSpeed);
  autoThrottle = 230;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("Turn: ");
  Serial.println(t);
  Serial.print("Original: ");
  Serial.println(turn);
  
  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t < 0) {
    autoSteerB = t_modifier;
  } else if (t > 0){
    autoSteerA = t_modifier;
  }

  Serial.print("SteerA: "); Serial.println(autoSteerA);
  Serial.print("SteerB: "); Serial.println(autoSteerB);
  Serial.print("Auto: "); Serial.println(autoThrottle);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);

  Serial.print("SpeedA: "); Serial.println(speedA);
  Serial.print("SpeedB: "); Serial.println(speedB);
  setSpeedMotorA(speedA);
  setSpeedMotorB(speedB);
}

void driveTo(struct GeoLoc &loc, int timeout) {
  ss.listen();
  GeoLoc cartLoc = checkGPS();
  bluetoothSerial.listen();

  if (cartLoc.lat != 0 && cartLoc.lon != 0) /*&& enabled*/ {
    float d = 0;
    //Start move loop here
    do {
      ss.listen();
      cartLoc = checkGPS();
      //Serial.print("loop\n");
      bluetoothSerial.listen();

      //Serial.print("help\n");
      d = geoDistance(cartLoc, loc);
      //Serial.print("please\n");
      float t = geoBearing(cartLoc, loc) - geoHeading();

      Serial.flush();

      Serial.print("Distance: ");
      Serial.println(geoDistance(cartLoc, loc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(cartLoc, loc));

      Serial.print("Heading: ");
      Serial.println(geoHeading());
      
      drive(d, t);
      timeout -= 1;
    } while (d > 3.0 && timeout>0);/*&& enabled */

    stopDriving();
  }
}

void setupCompass() {
  Compass.SetDeclination(-9, 38, 'W');
  Compass.SetSamplingMode(COMPASS_SINGLE);
  Compass.SetScale(COMPASS_SCALE_130);
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
}

void setup()
{
  // Compass
  Wire.begin();
  setupCompass();

  // Motor pins
  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);
  
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH);

  //Debugging via serial
  Serial.begin(9600);

  //GPS
  ss.begin(9600);

  //Bluetooth
  bluetoothSerial.begin(9600);
  Blynk.begin(bluetoothSerial, auth);
}

// Testing
void testDriveNorth() {
  float heading = geoHeading();
  int testDist = 10;
  Serial.println(heading);
  
  while(!(heading < 5 && heading > -5)) {
    drive(testDist, heading);
    heading = geoHeading();
    Serial.println(heading);
    delay(500);
  }
  
  stopDriving();
}

void loop()
{
  Blynk.run();
}
