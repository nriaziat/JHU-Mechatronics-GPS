#include "GPS.h"

GPS::GPS()
{
  _mode = COMMUNICATION_MODE::I2C;
}
GPS::GPS(COMMUNICATION_MODE mode)
{
  _mode = mode;
}

void GPS::begin() {
  if (_mode == COMMUNICATION_MODE::I2C){
    Wire.begin();
  } else {
    Serial1.begin(115200);
  }
}

robotPose GPS::getPose() {
  if (_mode == COMMUNICATION_MODE::I2C){
    Wire.requestFrom(_addr, _msg_size);    // request 29 bytes from device
    Wire.readBytes(_msg, _msg_size);
  } else {
    Serial1.write('p');
    while (Serial1.available() < 29);
    Serial1.readBytes(_msg, _msg_size);
  }
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
    byte bytes[4];
  } val, val2;
  val.a = dist;
  val2.a = angle;
  if (_mode == COMMUNICATION_MODE::I2C){
    Wire.beginTransmission(_addr);
    Wire.write(val.bytes, 4);
    Wire.write(val2.bytes, 4);
    Wire.endTransmission(_addr);
  } else {
    Serial1.write(val.bytes, 4);
    Serial1.write(",");
    Serial1.write(val2.bytes, 4);
  }
}