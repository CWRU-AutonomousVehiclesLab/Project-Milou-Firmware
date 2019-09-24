// This is a function-level outline of the SnowJoke firmware code
/*

The functionality we need is as follows:

	1. Stop the motors when the e-stop button is pressed
	2. RC control - Takes left and write motor speed as input, and passes it through to the motor controller
	3. PID - get current speed from the motor controller and check it against the input speed (in the case of driving on a hill or such)
	4. Firmware - takes X, Y and phi coordinates from the connected computer and moves the robot accordingly

*/
setup(){
	//start serial communication with the motor controller and computer
	//set constants and pinouts
	//write constants to serial outputs
	//call loops
	//global sets: SabertoothAddress, LeftMotorDirection, LeftMotorSpeed, RightMotorDirection, RightMotorSpeed
	//calls: RCPolling, ControlLoop, SabertoothMotorCommandLoop, SwitchRead
	//IO: Write to SaberToothSerial with setup values, setup pins as inputs and outputs
}

loop(){
	//global reads: LastSwitchReadTime, RCPollingLastTime,Switch_B, lastleftspeedrequest, requestflag,lastrightspeedrequest, lastresetcommand, ControlLoopLastTime, MotorCommandLastSent, Switch_C, LastDebug
	//calls: SwitchRead, RCPolling, leftspeedrequest, rightspeedrequest, getspeeddata, ControlLoop, SabertoothMotorCommandLoop, DebugPrint
}

RCPolling(){
	//global reads: ForwardCycle, AngularCycle, ForwardPulse, AngularPulse
	//global sets: RCForwardCommand, RCAngularCommand, SignedRCLeftSpeed, SignedRCRightSpeed, LeftTicksPerSecondDesiredRC, RightTicksPerSecondDesiredRC
	//IO: Write to WCL7 pin, read from PWMIN 0-5 registers
}

ControlLoop(){
	//global reads: Switch_A, Physical_Estop 
	//global sets: autonomous_enable, left_speed_error, left_speed_error_old, left_speed_error_sum, right_speed_error, right_speed_error_old, right_speed_error_sum
	//calls: RCControl, AutonomousPIDPassthrough
}

SabertoothMotorCommandLoop(){
	//global reads: Estopstatus, LeftMotorSpeed, RightMotorSpeed, LeftMotorDirection, RightMotorDirection, SabertoothAddress
	//global sets: LeftMotorSpeed, RightMotorSpeed, left_speed_error, left_speed_error_old, left_speed_error_sum, right_speed_error, right_speed_error_old, right_speed_error_sum, 
	//IO: output data to SaberToothSerial
}

SwitchRead(){
	//global sets: LastSwitchReadTime, Switch_A, Switch_B, Switch_C, Switch_D, Physical_Estop, Estopstatus
	//IO: read input pins WCH2, WBH3-WBH7
}

RCControl(){
	//global reads: LeftTicksPerSecondDesiredRC, RightTicksPerSecondDesiredRC, LeftTicksPerSecondEncoder, RightTicksPerSecondEncoder
	//global sets: LeftMotorDirection, LeftMotorSpeed, RightMotorDirection, RightMotorSpeed, LeftTicksPerSecondEncoderOld, RightTicksPerSecondEncoderOld
}

DebugPrint(){
	//global reads: autonomous_enable, RightEncoderTicks, LeftEncoderTicks, RightWheelSpeed, LeftWheelSpeed, desiredrightspeed, desiredleftspeed, Estopstatus
	//global sets: LastDebug	
}

AutonomousPIDPassthrough(){
	//global reads: desiredleftspeed, desiredrightspeed, EncoderRegister
	//global sets: LeftEncoderTicks, RightEncoderTicks, LeftTicksPerSecondEncoder, RightTicksPerSecondEncoder, LeftWheelSpeed, RightWheelSpeed, left_speed_error, right_speed_error, left_speed_error_old, right_speed_error_old, left_speed_error_sum, right_speed_error_sum, AutoLeftSpeed, AutoRightSpeed, LeftMotorSpeed, RightMotorSpeed
}

AutonomousNavigatakestion(){
	//global reads: ytarget, xtarget, y, x, waypointindex, phi, routecompleted, headingaligned, isrobotweighted
	//global sets: waypointindex, headingaligned, LeftMotorSpeed, RightMotorSpeed, routecompleted, forwardspeed, AutoLeftSpeed, AutoRightSpeed, LeftMotorDirection, RightMotorDirection
}

xrequest(){
	//global sets: lastxrequest, lastposrequest, requestflag
}

yrequest(){
	//global sets: lastyrequest, lastposrequest, requestflag
}

phirequest(){
	//global sets: lastphirequest, lastposrequest, requestflag
}

leftspeedrequest(){
	//global sets: lastleftspeedrequest, lastspeedrequest, requestflag
}

rightspeedrequest(){
	//global sets: lastrightspeedrequest, lastspeedrequest, requestflag
}

pathreset(){
	//global reads: Switch_D
	//global sets: lastresetcommand, desiredleftspeed, desiredrightspeed
}

getposdata(){
	//global reads: lastxrequest, lastyrequest,lastphirequest
	//global sets: requestflag, x, y, phi, lastxrequest, lastyrequest,lastphirequest
	//IO: Read from RasPiSerial
}

getspeeddata(){
	//global reads: lastleftspeedrequest, lastrightspeedrequest
	//global sets: requestflag, desiredleftspeed, desiredrightspeed, lastleftspeedrequest, lastrightspeedrequest
	//IO: Read from RasPiSerial
}
