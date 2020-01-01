#include <math.h>
#include <string.h>
//pins for RC input
#define FWDPULSEPIN 2   //RC Channel 2
#define ANGPULSEPIN 3   //RC Channel 1
#define ESTOPPULSEPIN 4 //RC Channel 3
//pins for physical switch inputs
#define SWITCHAPIN 7
#define SWITCHBPIN 8
#define SWITCHCPIN 9
#define SWITCHDPIN 10
#define ESTOPPIN 11
//pins for sabertooth serial communication
#define SABERTOOTHPINRX 0
#define SABERTOOTHPINTX 1
//pin for software enable output
#define SOFTWAREENABLEPIN 16
//pins for encoder input
#define LEFTENCODERPIN 17
#define RIGHTENCODERPIN 18

//positions for physical switch data in the Switches array
#define SWITCH_A 0  //autonomous enable
#define SWITCH_B 1  //Single/Double I mode
#define SWITCH_C 2  //Debug mode
#define SWITCH_D 3  //currently unused
#define SWITCH_ESTOP 4
//total number of RC channels
#define RCNUMBEROFCHANNELS 3
//for accessing the individual motor speeds in the MotorSpeeds array
#define LEFTSPEED 0
#define RIGHTSPEED 1
#define LEFTDIRECTION 2
#define RIGHTDIRECTION 3
//milliseconds between outputting debug info (1 hz)
#define DEBUGOUTPUTRATE 1000
//milliseconds between running the control loop (50 hz)
#define CONTROLLOOPRATE 20
//milliseconds between running the motor controller output loop (50 hz)
#define SABERTOOTHLOOPRATE 20
//max output to Sabertooth
#define SABERTOOTHMAX 127
//State definitions
#define STATE_ESTOP 0
#define STATE_RC 1
#define STATE_AUTONOMOUS 2
//encoder cycles per revolution of the wheel
#define ENCODERCYCLES 256
//constants for referencing values in the encoder structures
#define LEFTENCODER 0
#define RIGHTENCODER 1
//PID Gain Constants
#define KP 2  //proportional gain
#define KI 5  //integral gain
#define KD 1  //derivative gain
//PID Error Variables
float PIDError[2] = {0,0};
float PIDIntegralError[2] = {0,0};
float PIDDerivativeError[2] = {0,0};
float PIDPreviousError[2] = {0,0};
//data structures for calculating encoder cycles
uint32_t EncoderStartPulse[2];
uint32_t EncoderPulseData[2];

//for storing the motor speeds to be sent to the motor controller
uint32_t DesiredSpeeds[4];
//final speed after modification from the PID function with input from the encoder
uint32_t MotorSpeeds[4];
//for storing the states of the physical switches
boolean Switches[6];
//State variables
int State;
int NextState;

bool RCEStop;
//for timing of the PID
long PIDLastTime = 0;
//for timing of debug output
long LastDebug = 0;
//for timing of the main ControlLoop
long ControlLoopLastTime = 0;
//for timing of the Sabertooth controller output loop
long MotorCommandLastSent = 0;
//for communicating with the Sabertooth controller
int SabertoothAddress = B10000010;        // set Address to 130
int SabertoothMask = B01111111;

//kinematics calculations from old code
//uint32_t MaxRCDesiredTicksPerSecond = 51330; //(2 m/s) * (1 rev/0.957557m) * (24 motor rev/wheel rev) * (1024 ticks/motor rev)
//float TicksToMotorSpeedMultiplier = 0.00247419;// Max motor command of 127 divided by max desired ticks per second of 51330
//physical constants for calculating speeds
float WheelSpacing = 0.77; //Wheel spacing (center to center) in meters; assumed from previous code, double check
float MaxMotorRPM = 1;  //need this
float MotorRevsPerWheelRev = 24;  //assumed from previous code, double check
float WheelDiameter = 0.957557;   //assumed from previous code, double check
float MaxDesiredSpeed = 2;  //max speed of 2 m/s
//Global variables representing Autonomous commands (assuming for now it's a proportion -1 to 1)
float AutonomousLinearVelocity;
float AutonomousAngularVelocity;
//specifically for demoing the PID
float AutonomousTestTiming = 0;
float AutonomousTestAccumulator = 0;
//Channel-specific functions to calculate the PWM signal length
