# What is included?
The firmware code for Milou is seperated into the followings:

| File Name   	| Description  	|
|---	|---	|
| debug.ino  	|   This file includes all the information needed for debug purposes. <br> It include the prioritized message function and the periodic information dump function.	|
| encoderProcess.ino   	|  This file contains the encoder callback function. It updates the left and right encoder tick. 	|
| kinematics.ino  	|   This file includes all the kinematics calculation. Currently Milou uses Skid Steer (aka Roomba method). <br> **You will need to change this file if your robot is not skid steer.**  	|
| ledIndicator.ino  | This file contains the led indicator code. Basically, RGB means different operation mode. |
| MilouFirmware.ino | This file is the magical sheet for nearly 100 global variables. Have fun! Additionally, it also hosts ROS publisher, subscriber as well as ROS callback functions too.|
| pid.ino | This file contains the biggest work for the firmware, the PID system. If your robot is not Milou, you might want to go back to the master list of global variables and tune the PID parameters. |
| populateSpeed.ino | This file is used for converting RC pulses into linear and angular speed, as well as calculate the speed that is observed using wheel encoder. |
| rcProgress.ino | This file is used for storing the 3 RC interrupt to update the detected RC pulses. Additionally, this holds the function for updating the RC pulse from memory directly.|
| rosIO.ino | This file contains the functioin for publishing ROS topic. Currently only implemented to publish the basic robot telemetry.|
| main.ino | This file contains the loop() function where at a given rate, all the necessary functions are operated. |
| sabertoothWrite.ino | This file writes the sabertooth motor controller. This setup is very specific to the Sabertooth 2x60 motor controller. <br> Additionally, the digital logic killswitch for motor controller is also implemented in this file.|
| setup.ino | This file is the usual arduino setup section. All interrupts are attached in this file.  <br> **You should use this file for your reference on wiring the pins** <br> Additional pin map information can be found in MilouFirmware.ino|
| stateMachine.ino | This file includes the state machine for the robot. It is used for processing the Estop, RC, Autonomous State switch. <br> Currently, for safety, you have to be in the Estop State to switch between RC and Autonomous State.|
| switchProcess.ino | This is the switch reading interrupt callback. <br> Since we also read Estop board using this microcontroller, if detected Estop, a message will be sent via ROS interface to let upper controller know.|

## In summary
For Small modifications, refer to *MilouFirmware.ino*. To wire up the robot, use *MilouFirmware.ino* and *setup.ino*.

