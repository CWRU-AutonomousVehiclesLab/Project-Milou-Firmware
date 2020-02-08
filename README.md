# Project Milou Firmware Repo [![Build Status](https://travis-ci.com/CWRU-AutonomousVehiclesLab/Project-Milou-Firmware.svg?token=b4xHzfFvkzsycZ1PMx5q&branch=development)](https://travis-ci.com/CWRU-AutonomousVehiclesLab/Project-Milou-Firmware)  [![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://lbesson.mit-license.org/)


This repository contains code for Milou firmware that runs on Teensy 3.6. The teensy platform utilizes teensyduino IDE, a variation of the arduino. The system is setup to both operate under ROS enviornment as well as simple RC mode. You can also add in additional modes for dead reackoning feature.

## What is Project Milou?
Project Milou is Case Western Reserve University's second autonomous snowplow robot. The robot actively participate in the ION Autonomous Snowplow Challenge. Currently the latest revision of the robot is utilizing a Teensy 3.6, an ITX computer with a core i5 as its main control system.

## Logistics

**Latest revision is done at Febuary, 2020.**

This firmware is written by Frank (Chude Qian) [@frank-qcd-qk](https://github.com/frank-qcd-qk) with the help of code from Marc Krumbein (Father of Project Snowjoke) and [Drew Borneman](https://github.com/DrewBorneman). 

This project is maintained under CWRU Auontomous Vehicles Lab till 2021, and further maintained under CWRU Biologically Inspired Robotics (Biorobotics) Center. 

*Copyright belongs to CWRU Autonomous Vehicles lab, CWRU Biologically Inspired Robotics (Biorobotic) Center. Please use with acknowledgement.*

### Contact for questions
To contact email [Frank](mailto:cxq41@case.edu) if it is before 2021. If it is after 2021, email [Ian](mailto:ija2@case.edu) and [Dr. Merat](mailto:flm@case.edu). 

## Development Enviorment Setup
To develop teensy, you will need to use teensy duino, refer to [Teensy documentation](https://www.pjrc.com/teensy/td_download.html). This project uses Teensy 3.6 as its main development board. However, if you have a teensy 4.0 handy, you should be able to use that too, with some minor modifications and digging in the ROS Arduino Library. 

**NOTE: Currently, ROS Arduino does not officially support teensy 4.0**.

## Firmware documentation
You can find firmware documentation within project folder.

## Advanced Notes:
To use CI compile checker, you should always make sure all dependencies are latest. 

`travis.yaml` is responsible for setting up the virtual machine that is used for testing compile. You should make sure the teensyduino enviornment is correct as well as arduino ide are correct. 

**NOTE: If you have modified ROS message, you should regenerate the ROS library for arduino and update [this repo](https://github.com/CWRU-AutonomousVehiclesLab/ros-arduino-lib) as it is a dependency for the compile test. 4

`build-sketches.sh` is the file used for collecting all the files and compile test. You should not need to modify this.

## To-dos:
We have not yet implemented an onboard IMU as well as an onboard GPS system. If you are intereted, you can contact [me](mailto:cxq41@case.edu) to further discuss the implmentation.