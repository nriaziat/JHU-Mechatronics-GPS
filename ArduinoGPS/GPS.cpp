#include "GPS.h"

GPS::GPS()
{
}

void GPS::begin() {
  Wire.begin(_addr);
}

robotPose GPS::getPose() {
  delay(10);
  Wire.requestFrom(_addr, _msg_size);    // request 29 bytes from device
  Wire.readBytes(_msg, _msg_size);
  char *token;
  token = strtok(_msg, _delimiter);
  pose.x = atof(token);
  token = strtok(NULL, _delimiter);
  pose.y = atof(token);
  token = strtok(NULL, _delimiter);
  pose.theta = atof(token);
  return pose;
}


void GPS::updateOdometry(float dist, float angle){
  union {
    float a;
    unsigned char bytes[4];
  } val;
  val.a = dist;
  Wire.write(val.bytes, 4);
  val.a = angle;
  Wire.write(val.bytes, 4);
}