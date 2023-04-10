#include "IRCam.h"

IRCam::IRCam(float x_origin, float y_origin) {
  x1_origin = x_origin;
  y1_origin = y_origin;
}

IRCam::IRCam() {
  x1_origin = 0;
  y1_origin = 0;
  x2_origin = 0;
  y2_origin = 0;
}

void IRCam::begin() {
  Wire.begin();
  // Normal sensor initialize
  write2Byte(0x30, 0x01); delay(50);
  write2Byte(0x30, 0x08); delay(50);
  write2Byte(0x06, 0x90); delay(50);
  write2Byte(0x08, 0xC0); delay(50);
  write2Byte(0x1A, 0x40); delay(50);
  write2Byte(0x33, 0x33); delay(50);
  //experimental high sensitivity initialization
  //  IRCam::write2Byte(0x13, 0x04); delay(50);
  //  IRCam::write2Byte(0x1a, 0x04); delay(50);
  //  IRCam::write2Byte(0x30, 0x08); delay(50);
  //  IRCam::writeBlock1(); delay(50);
  //  IRCam::writeBlock2(); delay(50);
  //  IRCam::write2Byte(0x08, 0xC0); delay(50);
  //  IRCam::write2Byte(0x1A, 0x40); delay(50);
  //  IRCam::write2Byte(0x33, 0x33); delay(50);
  Serial.println("Set IRCam Parameters");
  delay(100);
}


void IRCam::write2Byte(byte d1, byte d2)
{
  Wire.beginTransmission(targetAddress);
  Wire.write(d1); delay(50); Wire.write(d2);
  Wire.endTransmission();
}

void IRCam::writeBlock1()
{
  Wire.beginTransmission(targetAddress);
  Wire.write(0x00);
  delay(50);
  Wire.write(0x00);
  delay(50);
  Wire.write(0x00);
  delay(50);
  Wire.write(0x00);
  delay(50);
  Wire.write(0x00);
  delay(50);
  Wire.write(0x00);
  delay(50);
  Wire.write(0x00);
  delay(50);
  Wire.write(0xFF);
  delay(50);
  Wire.write(0x00);
  delay(50);
  Wire.write(0x0C);
  Wire.endTransmission();
}

void IRCam::writeBlock2()
{
  Wire.beginTransmission(targetAddress);
  Wire.write(0x1a);
  delay(50);
  Wire.write(0x00);
  delay(50);
  Wire.write(0x00);
  delay(50);
  Wire.endTransmission();
}

void IRCam::read(){
  //IR sensor read
  int n_points = 4;
  Wire.beginTransmission(targetAddress);
  Wire.write(0x36);
  Wire.endTransmission();
  int msg_size = Wire.requestFrom(targetAddress, 16); // (MSB comes first)
  
  if (msg_size < 16){
    return;
  }

  for (i = 0; i < 16; i++) {
    data_buf[i] = 0;
  }
  i = 0;
  while (Wire.available() && i < 16) {
    data_buf[i] = Wire.read();
    i++;
  }

  int s;
  // Request the 2 byte heading
  Ix[0] = data_buf[1];
  Iy[0] = data_buf[2];
  s     = data_buf[3];
  Ix[0] += (s & 0x30) << 4;
  Iy[0] += (s & 0xC0) << 2;

  Ix[1] = data_buf[4];
  Iy[1] = data_buf[5];
  s     = data_buf[6];
  Ix[1] += (s & 0x30) << 4;
  Iy[1] += (s & 0xC0) << 2;

  Ix[2] = data_buf[7];
  Iy[2] = data_buf[8];
  s     = data_buf[9];
  Ix[2] += (s & 0x30) << 4;
  Iy[2] += (s & 0xC0) << 2;

  Ix[3] = data_buf[10];
  Iy[3] = data_buf[11];
  s     = data_buf[12];
  Ix[3] += (s & 0x30) << 4;
  Iy[3] += (s & 0xC0) << 2;

  n_points_found = 0;
  for (i = 0; i < n_points; i++) {
    if (Ix[i] != 1023 && Iy[i] != 1023 && Ix[i] != 0 && Iy[i] != 0){
      n_points_found++;
    }
  }
}

uint IRCam::getIx(uint* out){
  read();
  uint i = 0;
  for (auto val : Ix){
    out[i] = val;
    i++;
  }
  return n_points_found;
} 

uint IRCam::getIy(uint* out){
  read();
  uint i = 0;
  for (auto val : Iy){
    out[i] = val;
    i++;
  }
  return n_points_found;
}