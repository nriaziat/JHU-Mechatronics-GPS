/*
  IRCam.h - Library for using IR constellation for robot localization
  Created by Naveed D. Riaziat, January 2023
  Adapted from https://dfrobot.com by Lumi, Jan. 2014
*/

#ifndef IRCam_h
#define IRCam_h

#include <Arduino.h>
#include <Wire.h>
// #include <i2c_driver_wire.h>


struct robotPose {
  float x;
  float y;
  float theta;
};


// struct robotPose {
//   unsigned char x;
//   unsigned char y;
//   float theta;
// };


class IRCam
{
  public:
    IRCam(float x_origin, float y_origin);
    IRCam();
    void begin();
    uint getIx(uint* out);
    uint getIy(uint* out);

  private:
    void write2Byte(byte d1, byte d2);
    void writeBlock1();
    void writeBlock2();
    void read();

    const int IRsensorAddress = 0xB0;
    //int IRsensorAddress = 0x58;
    const int targetAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
    byte data_buf[16];
    int i;
    int Ix[4];
    int Iy[4];
    int n_points_found = 0; 
    float x1_origin = 0;
    float y1_origin = 0;
    float x2_origin = 0;
    float y2_origin = 0;
    const float pixel_scaler = 1;
};


#endif
