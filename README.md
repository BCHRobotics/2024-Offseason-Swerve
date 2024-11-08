# FIRST Robotics Team 2386 - Trojans: 2024 Offseason Swerve Drive

2386 Swerve Drive Code. Offseason project (March 2024 - January 2025).

This code is written solely for our 2024 FRC robot. 
Attempting to run the code on any other robot will require modifications.

## Project Goals

- Building and running autonomous paths in realtime
- Selecting and running autos based on an order of game pieces, not pre-built autos selected via drop-down
- Refining note detection in auto, allowing the robot to smoothly adjust position to pick up notes
- Refining tag detection in auto, allowing for smooth adjustment/scoring using apriltags
- Trajectory prediction, allowing the robot to predict where a note will land once shot
- Robot-aided shooting, allowing the robot to adjust position and score while moving
- Managing all robot functions on one controller

## IMPORTANT STUFF

- Label commits well, at least a few words explaining what you did
- Comment A LOT when/if possible, using [] to explain variable types and () for sidenotes/other important info

## Getting Started

### Installation

To run edit the code you must have the following items on your computer:
* WPILib VSCode: (https://github.com/wpilibsuite/allwpilib/releases)
* Java Tools: (https://code.visualstudio.com/docs/languages/java)

To drive the code you must have:
* FRC tools (Windows only, for driving): (https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html)

### Vendor Libraries

These are the vendor libraries that we are using. These are what you need to copy if they are missing.

```

https://dev.studica.com/releases/2024/NavX.json
https://software-metadata.revrobotics.com/REVLib-2024.json
https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
```

### Running the code

1. Clone or download this repo to your computer
2. Connect to the robot with one of the following
    * USB A to B
    * Wireless
    * Wired Ethernet
3. Right Click on **build.gradle** and click **Deploy Robot Code**
4. Launch Driver Station and Shuffleboard
5. Drive the Robot!

## License

FIRST BSD License

Copyright (c) 2009-2024 FIRST and other WPILib contributors 
Copyright (c) 2020-2024 Team 2386
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   Neither the name of FIRST, WPILib, nor the names of other WPILib contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
