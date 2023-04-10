#include "GPS.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

int encoder0PinA = 2;
int encoder0PinB = 3;
int encoder0Pos = 0;
int lastEncoder0Pos = 0;

GPS gps;
robotPose pose;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float orientation, last_orientation;
sensors_event_t orientationData;

void ISR1();
void ISR2();

void setup()
{
  Serial.begin(9600);  // start serial for output
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), ISR2, CHANGE);
  gps.begin();
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  last_orientation = orientationData.orientation.z;}

void loop()
{
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  orientation = orientationData.orientation.z;

  gps.updateOdometry(encoder0Pos - lastEncoder0Pos, orientation - last_orientation);
  lastEncoder0Pos = encoder0Pos;
  last_orientation = orientation;

  // Serial.println(encoder0Pos);

  pose = gps.getPose();
  Serial.print(pose.x);
  Serial.print(",");
  Serial.print(pose.y);
  Serial.print(",");
  Serial.println(pose.theta);
  // delay(50);
}

void ISR1(){
  int n2 = digitalRead(encoder0PinB);
  if (digitalRead(encoder0PinA)){
    if (n2) {
      encoder0Pos++;
    } else {
      encoder0Pos--;
    }
  }else{
    if (n2) {
      encoder0Pos--;
    } else {
      encoder0Pos++;
    }
  }
}

void ISR2(){
  int n1 = digitalRead(encoder0PinA);
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
