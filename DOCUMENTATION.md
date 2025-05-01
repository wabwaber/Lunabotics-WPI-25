# WPI Lunabotics MQP Code Documentation
Hello and welcome to this document where the team will be updating the documentation as we go along. This document is structured with a table of contents that should have links to jump to the appropriate section to make searching easy!

[psst hey you,  markdown by default doesn't support tabs so you have to use 4 non-blank special characters (&nbsp; or use %20 as that is the unicode character) to get around it. also for the table of contents use lists for each section]: #

## <center>**Table of Contents**</center>

### Section 1: Code Overview
- [Introduction](#overview-introduction)
- [Hardware Used](#hardware-used)
- [Libraries Used](#libs-used)

### Section 2: Jetson Code
- [IMU data ros bag](#imu-data-bag)
- [OpenVINSTest](#openVinsTest-pkg)

### Section 3: Arduino Code
- [Head.cpp](#head-cpp)
- [ArdyToMot.cpp](#ardy-to-mot-cpp)
- [Talon.cpp](#talon-cpp)

### Section 4: Raspberry Pi Code
- [Motor Communication Package](#motor_comm-pkg)
- [CAN Bus Motor Package](#can_bus_comm)
- [Controller Control Package](#controller_control_pkg)
- [Drivetrain Control Package](#drivetrain_control_pkg)
- [Jetson Communication Package](#jetson_control_pkg)


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
- Raspberry Pi 3 rev B01 [Datasheet](https://datasheets.raspberrypi.com/rpi3/raspberry-pi-3-b-plus-product-brief.pdf)

<a id="libs-used"></a>
### Libraries Used
- ROS2 Humble, galactic (Ubuntu 20.04)
- Docker [Home Page](https://www.docker.com/)
- OpenVINS [Home Page](https://docs.openvins.com/)
- inputs python lib [git](https://github.com/zeth/inputs)
- Raspberry PI I2C library [git](https://github.com/besp9510/pi_i2c)
- Raspberry PI wiring library [git](https://github.com/WiringPi/WiringPi)

## <center>Jetson Code</center>

<a id="imu-data-bag"></a>
### IMU data ros bag
If you intend on using the Realsense camera again, you may find the IMU data bag useful as it has around 2.5 hours of imu data on it. hopefully more later on but right now the Jetson is running off of a battery so it will have to make do for now

<a id="openVinsTest-pkg"></a>
### OpenVINS Test Package
This is a package that contains a few different things however they are all in service of providing a test environment for OpenVINS or open visual inertial navigation system. There are 3 files that I will be covering in the following subsections [bob.py](#bob-py), [setup.py](#setup-py), and [rvis_and_realsense.launch.py](#rvis-real-launch-py)
<a id="bob-py"></a>
### bob.py
The purpose of this file is explained in a metaphor at the top of the file. But in reality though I made this because we needed to remap the output topic from the realsense camera to the OpenVINS camera input topic. You would think ROS2 has this capability at runtime, not even in this outdated version but in the most recent but no it doesn't or if it does I did not find it online or in their documentation. So instead I spent my time, around a week, making my own node to do this for me, and thats how bob.py was formed.

Moving into the actual code, it starts with the required imports then we come across a class called bob, which is a ros2 node class. For the most part this information was taken from the 'Tutorials->Beginner:Client Libaries->Writing a simple publisher and subscriber (python)' page in the ros2 galactic documentation.the key part to all of this though is overriding the default quality of service profile. Firstly at the time of writing there is no documentation on how to do this at least I did not find it maybe on the 16th page of google I would have found it but I did not go past the 15th. Instead there is [this](#overriding-qos-settings) to explain the finer details and how its structured. First I made a QoS profile to hold the changes that I wanted to make, which in this case its changing the reliability to best effort the history is there only because the library would reject the profile if it wasn't.


<a id="overriding-qos-settings"></a>
### Overriding Quality of Service Settings ROS2
The code block below shows all that is needed in order to override a QoS setting(s). The first set of lines are just imports and an initializer for the profile that is written during the overwrite. In the initializer you put the policy you want to change in lower case then using the enum from rclpy for the policy you want to change. 

#### NOTE: It is REQUIRED to have the history policy, it will not work otherwise. 
```python
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
QoSOverride = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_ALL)
self.created_Subscription.qos_profile = QoSOverride
```
As seen in the above code block, it is written in python however the C++ code still has the same basic idea, creaitng a QoS profile and then adding it to the created subscription.


<a id="rvis-real-launch-py"></a>
### rvis_and_realsense.launch.py
This does the launching of the three nodes needed to test openVins. Those nodes being bob node, ov_msckf (OpenVINS), and the realsense camera node. It also changes the QoS so that OpenVINS can use the Realsense's data. There is also a full list of the realsense node's parameters which can be set in either terminal or in this launch file, where we have the IMU data stiched together and the rates set to be the same at 200hz. You cannot turn the gyroscope's rate down only up.

## <center>Arduino Code</center>
Clarification: We did not use an Arduino Mega for the final robot it was instead replaced with a Raspberry Pi. Sam has the explaination as to why so this section was written prior to that decision being made. I am leaving it here as useful documentation should you need uses for that code in the repo. If not go to [Section 4](#section-4-raspberry-pi-code), however there is a good bit of information that is in this section that is still useful, hence why it remains.

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

In an ideal world the loop would also handle the logic for turning, by passing it off to another cpp file to do either point turning or the spline math then having it return the required wheel values. The current commented out code handles just point turning where the right joy stick's x axis handles the wheel turning with the effort being just 100 * the input. 

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

After seeing all of these you may ask, why are you using PWM instead of the CAN bus that is obviously supporting 2 different sets of motors? well the motor controller that we have currently for the brushed turn motors requires a $500 board that is out of production and has no replacement. That is probably the reason why last years team did not use CAN bus exclusively. Again this is something that we want to fix but we would need funding of which would not be external. so we have limited options or we could use motors offered by Prof. Stafford.

The first function in this file is a set motors that takes in 4 integers representing the motor current value for each motor. This is also the function that the 2 input and 1 input versions send to, so I will not be talking more about those as they straight up just call this function. But it does the following steps
1. constrain all values relative to the max motor current for the drive motors, in case they somehow got above the maximum or below the negative of the max.
2. create the CAN bus packet which is a division of the given input divided by the conversion factor. which is something that is still unknown to us. As last year's didn't work and the default from the manual didn't work. so who know? But the second elements is the modulus of the value to the conversion factor. This is done for each value in a division then modulus division then modulus, etc. It also currently makes the assumption of the wheels going counter clockwise as we know the left side motors were the first two and the last two are the right side. but not which is which for front and back. So I assume its counter clockwise starting from the front left. 
3. A can frame is then created and sent with the result of the send saved as a variable
4. A debugging section where it prints the written packet to Serial so that we can see what it is sending.
5. Then the function returns a simple boolean on if the message was sent successfully or not.

The last set of functions are either redundant, talked about earlier, or not implemented.


## <center> Raspberry PI Code </center>
This section covers code written for the Raspberry PI. You may also notice its writing is of lower quality as this is written 3 hours before the project is due.

<a id="motor_comm-pkg"></a> 
### Motor Communication Package
This package handles the brushed motors, which use a PWM signal sent to the Talon motor controllers. If you look into it however, the Talons also support CAN. But in order to get that working the Talons require a configuration tool that costs $500 so we opted to not use that. Hence this package existing. 

Talon.cpp is the abstraction for the Talon motor controllers with a few basic functions and variables attached to the class instance. Motor_talk.cpp then takes this to another abstraction level where it can handle the PID controls for the motors, this is done through retrieving motor speeds from encoder_talk.cpp which abstracts the encoders, so rather than just getting the counts from this it calculates the speed, in degrees per second. 

Something you are probably wondering is why I put so many layers of abstraction between the drivetrain and the motors, well this is to, hopefully, assist you with reusing the code for the new robot. ideally you would not reuse this package though and would convert all the motors to CAN bus motors. But if not then do keep in mind that this code was not tested on the hardware so there will be issues.

<a id="can_bus_comm"></a>
### CAN Bus Motor Communication Package

This package is written in python as the CAN bus controller only has a library for python so this package exists, otherwise I would have integrated it into the motor_comm package. It is also not below the motor_comm node instead it is beside it in terms of the information transmission stack. The only other thing of interest is that it will also publish the motors speed, using the messages that the C620s send out which is something last years team didn't do and instead using encoders.

<a id="controller_control_pkg"></a>
### Controller Control Package

Written on the day of the CDR, so its ability to work is dubious at best. But the idea is that this runs along with the other motor communicators in order to manually control the drivetrain using the Logitech game controller. I would reccomend not using this package as there are quite a few things that need to be done to get it working, here is a list:
- ROS topics need to be setup for all motors
- Turning based on controller input
- Not reading all controller inputs instead creating a different function that will only pull those that are going to be used
- Setup the excavator controls
- Remove the while true loop from the main function area as it is not needed
- Make a launch file to launch the node

This list is not all encompassing and there are almost certainly things that I have missed. If you still want to use this as a basis then I would reccomend taking just the class for the logitech controller as that code is something I am somewhat proud of.

<a id="drivetrain_control_pkg"></a>
### Drivetrain Control Package
In short this whole package is just for the drivetrain controller. and only has two code files, one for the state machine and another for the header file which has a lot of variables and imports. Again there are holes in this as well, like the encoders not being read in as I was informed that the encoders were not going to be setup so I pivoted away from making sure those were programmed in. The state machine itself has some holes in it, this is because when I would start working on it I would think of a lower level system that had to be made so I would go make that, or I would be told that something in a lower level system that I had made has changed and needs to be changed.

<a id="jetson_control_pkg"></a>
### Jetson Communication Package
There are two major points in this package that need to be done, the first is getting it sending out the jetson command requests, and setup the WI-FI as that was something decided on the same day as the CDR so needless to say I did not have the time to set it up. A lot of the code that is written in this package is from Sam's github [link](https://github.com/thesamrooney/luna_control/blob/master/src/LunaController.cpp). 