#include "GPS.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

int encoder0PinA = 2;
int encoder0PinB = 3;
volatile int encoder0Pos = 0;
int lastEncoder0Pos = 0;
int encoder1PinA = 2;
int encoder1PinB = 3;
volatile int encoder1Pos = 0;
int lastEncoder1Pos = 0;
unsigned long t0, t1;

GPS gps(COMMUNICATION_MODE::I2C);
robotPose pose;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float orientation, last_orientation;
sensors_event_t orientationData;
imu::Vector<3> gyro;

void ISR1();
void ISR2();

const float CPR = 465;
const float WHEEL_DIAMETER = 7;

void setup()
{
  Serial.begin(115200);  // start serial for output
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), ISR2, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoder1PinA), ISR1, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoder1PinB), ISR2, CHANGE);
  gps.begin();
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // last_orientation = M_PI * orientationData.orientation.x / 180.f;
  t0 = millis();
  }

void loop()
{
  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // orientation = M_PI * orientationData.orientation.x / 180.f;
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  float dt = millis() - t0;
  orientation = gyro.z() * dt * 0.001 * M_PI / 180.f;
  float dx = WHEEL_DIAMETER * M_PI * float(encoder0Pos - lastEncoder0Pos) / CPR;

  if (fabs(dx) < 0.001 && fabs(orientation) < 0.001){
    Serial.print("Dx: ");
    Serial.print(dx);
    Serial.print(",");
    Serial.print("Dtheta: ");
    Serial.println(orientation);
  }

  gps.updateOdometry(dx, orientation);
  t0 = millis();
  lastEncoder0Pos = encoder0Pos;

  pose = gps.getPose();
  if (pose.x != 0 && pose.y !=0 && pose.theta != 0)
  {  
    Serial.print("Particle Filter Estimates: ");
    Serial.print(pose.x);
    Serial.print(",");
    Serial.print(pose.y);
    Serial.print(",");
    Serial.println(pose.theta);
  }
  delay(50);
}

void ISR1(){
  volatile int n2 = digitalRead(encoder0PinB);
  if (digitalRead(encoder0PinA)){
    if (n2) {
      encoder0Pos++;
    } else {
      encoder0Pos--;
    }
  } else{
    if (n2) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
  }
}

void ISR2(){
  volatile int n1 = digitalRead(encoder0PinA);
  if (digitalRead(encoder0PinB)){
    if (n1) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
  }else{
    if (n1) {
      encoder0Pos++;
    } else {
      encoder0Pos--;
    }
  }
}
