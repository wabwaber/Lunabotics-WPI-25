# WPI Lunabotics MQP Code Documentation
Hello and welcome to this document where the team will be updating the documentation as we go along. This document is structured with a table of contents that should have links to jump to the appropriate section making searching easy!

[psst hey you,  markdown by default doesn't support tabs so you have to use 4 non-blank special characters (&nbsp; or use %20 as that is the unicode character) to get around it. also for the table of contents use lists for each section]: #

## <center>**Table of Contents**</center>

### Section 1: Code Overview
- [Introduction](#overview-introduction)
- [Hardware Used](#hardware-used)
- [Libraries Used](#libs-used)

### Section 2: Jetson Code
- \{TODO}

### Section 3: Arduino Code
- [Head.cpp](#head-cpp)
- [ArdyToMot.cpp](#ardy-to-mot-cpp)
- [Talon.cpp](#talon-cpp)


## <center>Code Overview</center>
In this section we will be going over the broad strokes of how the code works as well as going over any related hardware or libraries. This will more than likely be a rehash of what is stated in the report. Maybe with a bit more depth as we can give as much detail as we want here!

<a id="overview-introduction"></a>
### Introduction
[talk about the broad algorithms used and how the code is structured.]: #

<a id="hardware-used"></a>
### Hardware Used
The Hardware referenced in this section is **ONLY** used for software related things so things like motor controllers, micro controllers, cameras. No motors, batteries, encoders, etc. will be found here. Also if any of the links are dead then you will have to find the document yourself. Hopefully I give you enough information to find it.

- Jetson Nano rev B01 [Manual](https://developer.download.nvidia.com/assets/embedded/secure/jetson/Nano/docs/NV_Jetson_Nano_Developer_Kit_User_Guide.pdf?IFNW4ZTb-H-qYxZfoi7MTdCb20kbF8c9X2fca6M_PHDMt3bN4nDS0O5v_YtL6nXPZ7so32MXn9XYN_yw6IorOUe1wHIc4w2p8Bj2606b13DIBbh9X-bvaFndv4VpX0S1I2eYV-A5hXi5UoYGkO1nUn-79RYcBsnZylgshBB9uHjuoHZCgeuOyjN7uXG6h7McgNw=&t=eyJscyI6IndlYnNpdGUiLCJsc2QiOiJkZXZlbG9wZXIubnZpZGlhLmNvbS9lbWJlZGRlZC1jb21wdXRpbmcifQ==)
- Arduino Mega 2560 Rev3 [Manual](https://docs.arduino.cc/resources/datasheets/A000067-datasheet.pdf)
- Intel Realsense D455 [Manual](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-D400-Series-Datasheet.pdf)


<a id="libs-used"></a>
### Libraries Used
- ROS2 Galactic (Ubuntu 20.04)
- OpenVINS [Home Page](https://docs.openvins.com/)
- inputs python lib [git](https://github.com/zeth/inputs)- ROS2 Humble maybe jazzy it really depends on how the jetson gets setup


## <center>Jetson Code</center>


## <center>Arduino Code</center>
In this section we will be going over each code file associated with the arduino mega and all its functions.

[Updated 11/12/2024]: #
<a id="head-cpp"></a> 
### Head.cpp
This is the head of the arduino and should be the file that is uploaded via PlatformIO. In its current state it has 5 global variables:
- A custom class object for the drive motors, see [ArdyToMot.cpp](#ardy-to-mot-cpp) for details
- An enumerator for the different robot states current at 4, one for stop one for driving forward, called drive, the last two are for the two different turning modes the first is turning on a point or a point turn, the other is turning around a point or the spline turn following
- the current state which is one of the above
- multiplier for the drive motors inputs which is a motor current up to a max of MAX_MOTOR_CURRENT_DRIVE or 12000. the reason as to why its multiplied by 2 is that the controller input gives a -0.5 to 0.5 for each axis so half of 2 times the max is just the max.
- finally is a currently unused boolean for if the serial connection is ready. Supposed to be used for the Arduino waiting on a signal from the python controller script.

Void setup() does was it says on the box. The baud rate is set to 115200 and the drive motor variable is initialized. Which currently begins the CAN bus communication and sets the motors to zero in case they start at some other non-zero value. It will, hopefully, in future initialize other drive motor relating things like turning and such but for now it does those two.

Void loop(), again does what it says on the box. But the current loop is this:
1. read in string from serial until it's the terminating character.
2. cast the string into a string (that is a char*) so allow for tokenization for the string. 
3. this then allows us to convert each given value from the token to a float and put it into an array called map that is the current input mapping of the controls
4. Lastly it sets the drive motors to the requested amount by multiplying the given input by the multiplier, as a reminder the multiplier is 2 times the max motor current. where the input given via serial or the python script is between -0.5 and 0.5

In an ideal world the loop would also handle the logic for turning, by passing it off to another cpp file to do either point turning or the spline math then having it return the required wheel values. The current commented stuff handles just point turning where the right joy stick's x axis handles the wheel turning with the effort being just 100 * the input. 

<a id="ardy-to-mot-cpp"></a> [Updated 11/12/2024]:#
### ArdyToMot.cpp
Ardy (Arduino) to motor is the primary handler for the drive train system. Which I will be calling drive wheels or wheels. But it encapsulates both the drive wheel motors and the turning motors. The number for global variables is ridiculous and they should be moved into the .h file but I haven't done that yet because it is likely that this is going to be revamped soon. But here are all the current global variables
- left side pwm pin for the turning motor
- right side pwm pin for the turning motor
- CAN bus chip select pin
- CAN bus data input pin
- CAN bus data output pin
- CAN bus clock input pin
- CAN bus interrupt pin (make sure this one is plugged into a pin on the arduino that supports interrupts!)
- CAN bus drive motors ID, 512 in decimal and 0x200 in hexadecimal
- CAN bus conveyor motor ID, 511 in decimal and 0x1FF in hexadecimal
- Drive motors message length, as in how many elements of the array there are not the memory size of the message
- CAN bus conveyor message length
- CAN bus bit rate, currently set to 8MHz at 1Mbps
- turning error for left turn motor
- turning error for right turn motor
- goal turn angle for left motor
- goal turn angle for right motor
- PWM object for left side turn motor
- PWM object for right side turn motor
- CAN bus communication board object might be replaced with a different library

After seeing all of these you may ask, why are you using PWM instead of the CAN bus that is obviously supporting 2 different sets of motors? well the motor controller that we have currently for the brushed turn motors requires a $500 board that is out of production and has no replacement. That is probably the reason why last years team did not use CAN bus exclusively. Again this is something that we want to fix but we would need funding of which would be not external. so we have limited options or we could use motors provided by Prof. Stafford.

The first function in this file is a set motors that takes in 4 integers representing the motor current value for each motor. This is also the function that the 2 input and 1 input versions send to, so I will not be talking more about those as they straight up just call this function. But it does the following steps
1. constrain all values relative to the max motor current for the drive motors, in case they somehow got above the maximum or below the negative of the max.
2. create the CAN bus packet which is a division of the given input divided by the conversion factor. which is something that is still unknown to us. As last year's didn't work and the default from the manual didn't work. so who know? But the second elements is the modulus of the value to the conversion factor. This is done for each value in a division then modulus division then modulus, etc. It also currently makes the assumption of the wheels going counter clockwise as we know the left side motors were the first two and the last two are the right side. but not which is which for front and back. So I assume its counter clockwise starting from the front left. 
3. A can frame is then created and sent with the result of the send saved as a variable
4. A debugging section where it prints the written packet to Serial so that we can see what it is sending.
5. Then the function returns a simple boolean on if the message was sent successfully or not.

The last set of functions are either redundant, talked about earlier, or not implemented.

<a id="talon-cpp"></a> [Updated: 11/12/2024]:#
### Talon.cpp

