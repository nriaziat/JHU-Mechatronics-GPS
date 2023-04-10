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
- Update odometry with `gps.updateOdometry(ditance, angle);`
- Get the updates pose with  `gps.getPose();`. It returns a robotPose structure.
- The robotPose structure includes three members: `x, y, theta` in the world frame.




# Structure

The structure of your new Arduino project is as follows:

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
