#ifndef GPS_h
#define GPS_h

#include <Arduino.h>
#include <Wire.h>

struct robotPose {
  float x;
  float y;
  float theta;
};


class GPS
{
  public:
    GPS();
    void begin();
    robotPose getPose();
    void updateOdometry(float dist, float angle);

  private:
    static const int _msg_size = 29;
    char _msg[_msg_size];
    const char *_delimiter = ",";
    const int _addr = 8;
    robotPose pose;
};

#endif
