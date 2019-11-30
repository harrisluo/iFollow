#define BLYNK_USE_DIRECT_CONNECT
//LEFT SIDE MOTORS
#define L_ENA 3
#define L_IN1 2
#define L_IN2 4

//RIGHT SIDE MOTORS
#define R_ENB 5
#define R_IN3 6
#define R_IN4 7

//DIRECTION CONVENTION
#define FORWARD 1
#define LEFT 2
#define STOP 3
#define RIGHT 4
#define BACKWARD 5

//ULTRASONIC PINS
#define TRIG_PIN1 8
#define ECHO_PIN1 9

#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#define BLUETOOTH_TX_PIN 10
#define BLUETOOTH_RX_PIN 11

int switch1;
int switch2;
int switch3;
int switch4;
int driveDirection = STOP;
int driveSpeed = 100;
char auth[] ="3cIMzOvMabyrD2jCGUfv_q-MrHU-ydPM";

// Serial components
SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);


void setup()
{
  //Set motor control pins as output
  pinMode(L_ENA, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_ENB, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);

  //ULTRASONIC TEST
  pinMode(13, OUTPUT);
  
  //Debugging via serial
  Serial.begin(9600);

  //Bluetooth
  bluetoothSerial.begin(9600);
  Blynk.begin(bluetoothSerial, auth);
  //pinMode(13, OUTPUT);
}

void forward() {
  //Set left direction forward
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  // set speed
  analogWrite(L_ENA, driveSpeed);
  //Set right direction forward
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, HIGH);
  // set speed
  analogWrite(R_ENB, driveSpeed);
  delay(500);
}

void left() {
  //Set left direction forward
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  // set speed
  analogWrite(L_ENA, driveSpeed);
  //Set right direction backward
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
  // set speed
  analogWrite(R_ENB, driveSpeed);
  delay(500);
}

void turnLeft() {
  //Set left direction forward
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  // set speed
  analogWrite(L_ENA, driveSpeed/4);
  //Set right direction forward
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, HIGH);
  // set speed
  analogWrite(R_ENB, driveSpeed);
  delay(500);
}

void right() {
  //Set left direction backward
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  // set speed
  analogWrite(L_ENA, driveSpeed);
  //Set right direction forward
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, HIGH);
  // set speed
  analogWrite(R_ENB, driveSpeed);
  delay(500);
}

void turnRight() {
  //Set left direction forward
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  // set speed
  analogWrite(L_ENA, driveSpeed);
  //Set right direction forward
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, HIGH);
  // set speed
  analogWrite(R_ENB, driveSpeed/2);
  delay(500);
}

void backward() {
  //Set left direction backward
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  // set speed
  analogWrite(L_ENA, driveSpeed);
  //Set right direction backward
  digitalWrite(R_IN3, HIGH);
  digitalWrite(R_IN4, LOW);
  // set speed
  analogWrite(R_ENB, driveSpeed);
  delay(500);
}

void driveStop() {
  //Set left direction stopped
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  // set speed
  analogWrite(L_ENA, driveSpeed);
  //Set right direction stopped
  digitalWrite(R_IN3, LOW);
  digitalWrite(R_IN4, LOW);
  // set speed
  analogWrite(R_ENB, driveSpeed);
  delay(500);
}

void obstacleManeuver() {
  backward();
  backward();
  right();
}

float ultrasonicDist() {
  float duration, distance = 0;
  //TEST
  
  // Write a pulse to the HC-SR04 Trigger Pin
  digitalWrite(TRIG_PIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);
 
  // Measure the response from the HC-SR04 Echo Pin
  duration = pulseIn(ECHO_PIN1, HIGH);
  Serial.println(duration);
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
 
  distance = (duration / 2) * 0.0343;
  Serial.println(distance);
  return distance;
}

BLYNK_WRITE(V1) {
 switch1 = param.asInt();
 if(switch1 == 1) {
  /*if(ultrasonicDist() < 10) {
    digitalWrite(13, HIGH);
    obstacleManeuver();
  } 
  else 
  {
    digitalWrite(13, LOW);*/
    forward();
  //}
 }
 if(switch1 == 0 && switch2 == 0 && switch3 == 0 && switch4 == 0 )
 {
  digitalWrite(13, LOW);
  driveStop();
 }
} 

BLYNK_WRITE(V2) {
 int switch2 = param.asInt();
 if (switch2 == 1) {
  turnRight(); //left();
  }
 if(switch1 == 0 && switch2 == 0 && switch3 == 0 && switch4 == 0 )
 {
  driveStop();
 }
} 

BLYNK_WRITE(V3) {
 switch3 = param.asInt();
 if(switch3 == 1) {
  turnLeft(); //right();
 }
 if(switch1 == 0 && switch2 == 0 && switch3 == 0 && switch4 == 0 )
 {
  driveStop();
 }
} 

BLYNK_WRITE(V4) {
 int switch4 = param.asInt();
 if(switch4 == 1) {
  backward();
 }
 if(switch1 == 0 && switch2 == 0 && switch3 == 0 && switch4 == 0 )
 {
  driveStop();
 }
} 

BLYNK_WRITE(V5){
  int s = param.asInt();
  driveSpeed = s;
}

void loop()
{
  Blynk.run();
}
