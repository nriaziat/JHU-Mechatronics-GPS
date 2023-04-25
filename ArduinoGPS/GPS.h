#ifndef GPS_h
#define GPS_h

#include <Arduino.h>
#include <Wire.h>

struct robotPose {
  float x;
  float y;
  float theta;
};

enum class COMMUNICATION_MODE {
  SER,
  I2C
};

class GPS
{
  public:
    GPS();
    GPS(COMMUNICATION_MODE mode);
    void begin();
    robotPose getPose();
    void updateOdometry(float dist, float angle);

  private:
    COMMUNICATION_MODE _mode;
    static const int _msg_size = 29;
    char _msg[_msg_size];
    const char *_delimiter = ",";
    const int _addr = 8;
    robotPose pose;
};

#endif
