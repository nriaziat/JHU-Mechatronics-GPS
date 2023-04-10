# JHU EN.530.421 GPS Code

This library interfaces a [SEN0158](https://www.dfrobot.com/product-1088.html) IR sensor, Teensy 4.0, and Arduino Mega to localize a robot within the final project workspace. The SEN0158 and Teensy 4.0 communicate over I2C continuously, while odometry updates and pose estimate updates are communicated to the Arduino Mega from the Teensy by I2C request.

# Dependencies

* Arduino IDE >= 1.5

# Usage

To get started clone this repo or download zip and extract on your local machine. To clone using git:

```shell
$ git clone https://github.com/nriaziat/JHU-Mechatronics-GPS.git
```
In the Arduino IDE, go to Sketch->Include Library->Add .ZIP Library..

Navigate to the ArduinoGPS.zip file in your newly cloned directory and install.

Once the library is installed, you can include the appropriate headers for the Mega by adding the "GPS.h" header to your code.

- `#include "GPS.h` must be at the top of your main .ino file
- Instantiate a GPS object like: `GPS gps`
- Start the GPS object in your with `gps.begin()`
- Update odometry with `gps.updateOdometry(distance, angle);`
- The distance and angles updates should be given in centimeters and radians relative to the last update.
- Get the updates pose with  `gps.getPose();`. It returns a robotPose structure.
- The robotPose structure includes three members: `x, y, theta` in centimeters/radians in the world frame.
- The world frame is aligned with the marked corner of the field with the y-axis along the short edge and angles measured counter-clockwise from the x-axis.

## Hardware and Electronics

The Teensy's Vin port should be connected to +5 V and ground to the Arduino ground. The SDA1 and SCL1 ports on the Teensy (pins 17 and 16) should be connected to SDA and SCL on the Arduino Mega. The camera itself needs +5 V power and ground as well as its SDA and SCL connected to SDA0 and SCL0 on the Teensy (pins 19 and 18).

The camera's top surface should be mounted precisely at 6 cm from the ground. 

# Structure

The structure of this project is as follows:

```shell
.
├── ArduinoGPS                        Arduino MEGA GPS Library
│   └── GPS.h                         Arduino Source File
│   └── GPS.cpp                       Arduino Source File
│   └── ArduinoGPS.ino                Arduino Usage Example
├── constellationGPS                  Main sketch file
│   └── IRCam.h                       Teensy Source File
│   └── IRCam.cpp                     Teensy Source File
│   └── ParticleFilter.cpp            Teensy Source File
│   └── ParticleFilter.h              Teensy Source File
│   └── ParticleFilter.ino            Teensy Main Sketch
├── ProcessingVisualizer              Processing Debugger
│   └── readConstellationGPS.pde      Main Processing sketch
├── ArduinoGPS.zip                    Library Folder
├── constellationGPS.zip              Library Folder
├── LICENCE.txt                       Licence file
└── README.md                         Project README file
```
