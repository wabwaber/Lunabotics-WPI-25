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

<a id="head-cpp"></a>
### Head.cpp


<a id="ardy-to-mot-cpp"></a>
### ArdyToMot.cpp


<a id="talon-cpp"></a>
### Talon.cpp
