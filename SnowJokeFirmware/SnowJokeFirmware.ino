#include <math.h>
#include <string.h>
//to-do: check the FwdCycle, AngCycle, EStopCycle values in the old code and see if they're necessary for safety

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
//positions for pin PWM data in the RCPulseData array
#define FWDPULSEDATA 0
#define ANGPULSEDATA 1
#define ESTOPPULSEDATA 2
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
//holds the temporary start of signal data, for use in timing the PWM width of the RC input signals
uint32_t RCStartPulse[RCNUMBEROFCHANNELS];
//holds data from the RC PWM inputs
uint32_t RCTempPulseData[RCNUMBEROFCHANNELS];
//more permenant pulse data copied over from RCTempPulseData
uint32_t RCPulseData[RCNUMBEROFCHANNELS];
//for storing the motor speeds to be sent to the motor controller
uint32_t MotorSpeeds[4];
//for storing the states of the physical switches
boolean Switches[6];
//State variables
int State;
int NextState;

bool RCEStop;
//for timing of debug output
long LastDebug = 0;
//for timing of the main ControlLoop
long ControlLoopLastTime = 0;
//for timing of the Sabertooth controller output loop
long MotorCommandLastSent = 0;
//for communicating with the Sabertooth controller
int SabertoothAddress = B10000010;        // set Address to 130
int SabertoothMask = B01111111;
//checksums for outputting the motor controller data
int  LeftChecksum = (SabertoothAddress + MotorSpeeds[LEFTDIRECTION] + MotorSpeeds[LEFTSPEED]); // Check left motor commands against this
int  RightChecksum = (SabertoothAddress + MotorSpeeds[RIGHTDIRECTION] + MotorSpeeds[RIGHTSPEED]); // Check right motor commands against this
//constants relating to the RC functioning
//! revisit these numbers, first four likely wrong
uint32_t RCCenter = 1500;
uint32_t RCMax = 2300;
uint32_t RCMin = 700;
uint32_t RCTimeout = 1930000;
uint32_t EStopThreshold = 1300;
//uint32_t MaxRCDesiredTicksPerSecond = 51330; //(2 m/s) * (1 rev/0.957557m) * (24 motor rev/wheel rev) * (1024 ticks/motor rev)
//float TicksToMotorSpeedMultiplier = 0.00247419;// Max motor command of 127 divided by max desired ticks per second of 51330
//physical constants for calculating speeds
float WheelSpacing = 0.77; //Wheel spacing (center to center) in meters; assumed from previous code, double check
float MaxMotorRPM = 1;  //need this
float MotorRevsPerWheelRev = 24;  //assumed from previous code, double check
float WheelDiameter = 0.957557;   //assumed from previous code, double check
float MaxDesiredSpeed = 2;  //max speed of 2 m/s

//Channel-specific functions to calculate the PWM signal length
void FwdPulse() {
  if (digitalRead(FWDPULSEPIN)) {
    RCStartPulse[FWDPULSEDATA] = micros();
    //Serial.println("fwd pulse on")
  }
  else {
    RCTempPulseData[FWDPULSEDATA] = micros() - RCStartPulse[FWDPULSEDATA];
    //Serial.println("fwd pulse off");
  }
  return;
}

void AngPulse() {
  if (digitalRead(ANGPULSEPIN)) {
    RCStartPulse[ANGPULSEDATA] = micros();
    //Serial.println("ang pulse on");
  }
  else {
    RCTempPulseData[ANGPULSEDATA] = micros() - RCStartPulse[ANGPULSEDATA];
    //Serial.println("ang pulse off");
  }
  return;
}

void EStopPulse() {
  if (digitalRead(ESTOPPULSEPIN)) {
    RCStartPulse[ESTOPPULSEDATA] = micros();
    //Serial.println("estop pulse on");
  }
  else {
    RCTempPulseData[ESTOPPULSEDATA] = micros() - RCStartPulse[ESTOPPULSEDATA];
    //Serial.println("estop pulse off");
  }
  return;
}

//Copies over the entire temp array to a more permanant one before accessing so an interrupt won't change values while it's being accessed
void GetRCData() {
  noInterrupts(); //pauses interrupts while reading data
  memcpy((void *)RCPulseData, (const void *)RCTempPulseData, sizeof(RCTempPulseData));
  interrupts(); //resumes other interrupts
  return;
}

void RCControl() {
  GetRCData();
  uint32_t ForwardPulse = RCPulseData[FWDPULSEDATA];
  uint32_t AngularPulse = RCPulseData[ANGPULSEDATA];
  uint32_t EStopPulse = RCPulseData[ESTOPPULSEDATA];
  long forwardPercent;
  long angularPercent;
  //if EStop Pulse > 1300, disable the software enable output
  if (EStopPulse > EStopThreshold) {
    State = STATE_ESTOP;
    NextState = STATE_ESTOP;
    digitalWrite(SOFTWAREENABLEPIN, 1); //output low when in EStop State
    RCEStop = true;
    MotorSpeeds[LEFTSPEED] = 0;
    MotorSpeeds[RIGHTSPEED] = 0;
    MotorSpeeds[LEFTDIRECTION] = 0;
    MotorSpeeds[RIGHTDIRECTION] = 0;
    return;
  }
  else{
    if(RCEStop == true){    //we're just leaving RC EStop
      if(Switches[SWITCH_ESTOP]==0)
        NextState = STATE_ESTOP;
      else if(Switches[SWITCH_A]==0)
        NextState = STATE_AUTONOMOUS;
      else
        NextState = STATE_RC;
    }
    RCEStop = false;
    if (State == STATE_RC){    //only continue on if in RC state
      forwardPercent = map(ForwardPulse, RCMin, RCMax, -1, 1);
      angularPercent = map(AngularPulse, RCMin, RCMax, -1, 1);
      ConvertRCToMotorSpeeds(forwardPercent, angularPercent);
    }
  }
  return;
}

void ConvertRCToMotorSpeeds(uint32_t forwardPercent, uint32_t angularPercent) {
  float leftPercentage;
  float rightPercentage;
  long leftMotorSpeed;
  long rightMotorSpeed;
  long leftMotorDirection;
  long rightMotorDirection;
  long maxPossibleSpeed;
  float speedCapPercent;
  //maybe do some sort of validation, but shouldn't have to because interrupts
  //convert from percentage of linear and angular velocity to percentage of left and right wheel
  leftPercentage = (forwardPercent - ((WheelSpacing / 2)*angularPercent))/(1+(WheelSpacing / 2));   
  rightPercentage = (forwardPercent + ((WheelSpacing / 2)*angularPercent))/(1+(WheelSpacing / 2)); 
  //now we have a percentage from -1 to 1 for how fast each wheel should be going
  
  //calculate the output to the motor controller, which is from -127 to 127
  //first, we'll get a multiplier so we don't command our wheels to go faster than the max desired speed
  //to do that we need to calculate the maximum possible speed of the robot
  maxPossibleSpeed = MaxMotorRPM * (1/MotorRevsPerWheelRev) * WheelDiameter;
  //now get the speed cap as a percent of the maximum
  //speedCapPercent = MaxDesiredSpeed/maxPossibleSpeed;
  //until we know the constants, just output motor commands at half of max possible speed
  speedCapPercent = 0.5;
  //finally, get the values we're outputting to the motor controller by multiplying these all together
  leftMotorSpeed = leftPercentage * SABERTOOTHMAX * speedCapPercent;
  rightMotorSpeed = leftPercentage * SABERTOOTHMAX * speedCapPercent;
  /*SignedLeftSpeed = ((RCCenter - ForwardPulse) - 1.5 * WheelSpacing * (RCCenter - AngularPulse));
  SignedRightSpeed = ((RCCenter - ForwardPulse) + 1.5 * WheelSpacing * (RCCenter - AngularPulse));

  LeftMotorSpeed = TicksToMotorSpeedMultiplier * (((MaxRCDesiredTicksPerSecond) * SignedLeftSpeed) / (RCMax - RCCenter));
  RightMotorSpeed = TicksToMotorSpeedMultiplier * (((MaxRCDesiredTicksPerSecond) * SignedRightSpeed) / (RCMax - RCCenter));
  */
  //encode signs in seperate direction variable; remove from speed
  if (leftMotorSpeed < 0)
  {
    leftMotorDirection = B00000000;
  }
  else if (leftMotorSpeed >= 0)
  {
    leftMotorDirection = B00000001;
  }
  if (rightMotorSpeed < 0)
  {
    rightMotorDirection = B00000100;
  }
  else if (rightMotorSpeed >= 0)
  {
    rightMotorDirection = B00000101;
  }

  /*
  //Ensure motor speeds are equal to or below 127 and ensure turning is possible
  if ((leftMotorSpeed > 127) || (rightMotorSpeed > 127))
  {
    if (leftMotorSpeed > rightMotorSpeed)
    {
      rightMotorSpeed = ((rightMotorSpeed * 127) / leftMotorSpeed); //Keep the same speed ratio but adjust maximum to 127
      leftMotorSpeed = 127;
    }
    else if (leftMotorSpeed < rightMotorSpeed)
    {
      leftMotorSpeed = ((leftMotorSpeed * 127) / rightMotorSpeed);
      RightMotorSpeed = 127;
    }
    else if (leftMotorSpeed == rightMotorSpeed)
    {
      leftMotorSpeed = 127;
      RightMotorSpeed = 127;
    }
    */
  //put into the array for accessing elsewhere
  MotorSpeeds[LEFTSPEED] = abs(leftMotorSpeed);
  MotorSpeeds[RIGHTSPEED] = abs(rightMotorSpeed);
  MotorSpeeds[LEFTDIRECTION] = leftMotorDirection;
  MotorSpeeds[RIGHTDIRECTION] = rightMotorDirection;

  return;
  }

  //read the values of all switches
  void ReadSwitches()
  {
    //State = NextState;
    //HardEStop = NextHardEStop;
    Switches[SWITCH_A] = digitalRead(SWITCHAPIN);
    Switches[SWITCH_B] = digitalRead(SWITCHBPIN);
    Switches[SWITCH_C] = digitalRead(SWITCHCPIN);
    Switches[SWITCH_D] = digitalRead(SWITCHDPIN);
    Switches[SWITCH_ESTOP] = digitalRead(ESTOPPIN);
    
    if(Switches[SWITCH_ESTOP]== 0){
      State = STATE_ESTOP;
      digitalWrite(SOFTWAREENABLEPIN, 0); //output low when in EStop State
    }
    else{
      State = NextState;
      digitalWrite(SOFTWAREENABLEPIN, 1); //output high when not in EStop State
    }
    /*if (State==STATE_ESTOP || Switches[SWITCH_ESTOP]==0) { //All the EStop scenarios
      if(Switches[SWITCH_ESTOP]==0){  //The button's pressed, hard EStop regardless
        NextState = STATE_ESTOP;
        NextHardEStop = true;
        digitalWrite(SOFTWAREENABLEPIN, 1); //output high when in EStop State
      }
      else if(HardEStop==true && Switches[SWITCH_ESTOP]==1 && RCEStop == false){  //the button's just been let up after being in hard EStop, and RC EStop isn't on
        digitalWrite(SOFTWAREENABLEPIN, 0); //output low when not in EStop State
        NextHardEStop = false;
        if(Switches[SWITCH_A] == 0)
          NextState = STATE_AUTONOMOUS;
        else
          NextState = STATE_RC;
      }
      else{ //no change, stay in soft EStop
        NextState = STATE_ESTOP;
        NextHardEStop = false;
        digitalWrite(SOFTWAREENABLEPIN, 1); //output low when in EStop State
      }
    }
    else if((State==STATE_RC && Switches[SWITCH_A]==0)||(State==STATE_AUTONOMOUS && Switches[SWITCH_A]==1)){   //if Switch A has been flipped (0 = autonomous, 1 = RC), put in soft EStop
        NextState = STATE_ESTOP;
        NextHardEStop = false;
        digitalWrite(SOFTWAREENABLEPIN, 1); //output low when in EStop State
    }
    else //no change
      NextState = State;  
      */
    return;
  }

  void SabertoothMotorCommandLoop()
    {
      byte addressByte;
      byte leftCommandByte;
      byte leftDataByte;
      byte leftChecksumByte;
      byte rightCommandByte;
      byte rightDataByte;
      byte rightChecksumByte;
      
    MotorCommandLastSent = millis();
    
  
    //If robot is estopped, set the speed to 0 for outputting
    if (State == STATE_ESTOP)
    {
      MotorSpeeds[LEFTSPEED] = B00000000;
      MotorSpeeds[RIGHTSPEED] = B00000000;
      
    }

    //We're using the packetized serial mode to interface with the Sabertooth controller
    //This is a four byte packet:
    //byte 1: address of the controller, set by DIP switches 4, 5 and 6 on the controller
    addressByte = SabertoothAddress;
    //byte 2: the command, relevant ones here are 0 = motor 1 forward, 1 = motor 1 backwards, 4 = motor 2 forward, 5 = motor 2 backwards, they've already been encoded
    leftCommandByte = MotorSpeeds[LEFTDIRECTION];
    rightCommandByte = MotorSpeeds[RIGHTDIRECTION];
    //byte 3: the data, a speed from 0 - 127
    leftDataByte = MotorSpeeds[LEFTSPEED];
    rightDataByte = MotorSpeeds[RIGHTSPEED];
    //byte 4: the checksum, the sum of the previous 3 bytes, and ANDed with the mask 01111111b 
    leftChecksumByte = (addressByte + leftCommandByte + leftDataByte) & SabertoothMask;
    rightChecksumByte = (addressByte + rightCommandByte + rightDataByte) & SabertoothMask;

    //output our packets, starting with left
    Serial1.write(addressByte);
    Serial1.write(leftCommandByte);
    Serial1.write(leftDataByte);
    Serial1.write(leftChecksumByte);
    //right packet
    Serial1.write(addressByte);
    Serial1.write(rightCommandByte);
    Serial1.write(rightDataByte);
    Serial1.write(rightChecksumByte);
    return;
    }
    
  //the state changes are all handled in the switch read function, so we use this to take the appropriate action
  void ControlLoop()
  {
    ControlLoopLastTime = millis(); // time the interval of ControlLoop running
    switch(State){
      case STATE_ESTOP:
        //set the next state to the switch positions if in hard estop
        if(Switches[SWITCH_ESTOP]==0){
          if(Switches[SWITCH_A]==0) //low - autonomous enable
            NextState = STATE_AUTONOMOUS;
          else
            NextState = STATE_RC;
        }
        break;
      case STATE_RC:
        //nothing here now that we moved the RC control to the main loop
      case STATE_AUTONOMOUS:
        //autonomous code
        break;
    }
    return;
  }

  void DebugOutput(String outputString, int level, bool debug, bool minimal)
  {
    switch (level)
    {
      case 0:
        if (debug) {
          Serial.print("[DEBUG]");
        }
        break;
      case 1:
        if (!minimal) {
          Serial.print("[INFO]");
        }
        break;
      case 2:
        Serial.print("[WARNING]");
        break;
      case 3:
        Serial.print("[ERROR]");
        break;
      case 4:
        Serial.print("[CRITICAL]");
        break;
      case 5:
        Serial.print("[FATAL]");
        break;
    }
    Serial.println(outputString);

    return;
  }


  void DebugPrint()
  {
    String OutputString = "";
    LastDebug = millis();
    //put debug info here
    OutputString = "State: " + String(State);
    DebugOutput(OutputString, 0, true, false);
    OutputString = "Switches (A, B, C, D, ESTOP): " + String(Switches[SWITCH_A]) + ", " + String(Switches[SWITCH_B]) + ", " + String(Switches[SWITCH_C]) + ", " + String(Switches[SWITCH_D]) + ", " + String(Switches[SWITCH_ESTOP]);
    DebugOutput(OutputString, 0, true, false);
    if(State==STATE_RC){
      OutputString = "Forward Pulse: " + String(RCPulseData[FWDPULSEDATA]);
      DebugOutput(OutputString, 0, true, false);
      OutputString = "Ang Pulse: " + String(RCPulseData[ANGPULSEDATA]);
      DebugOutput(OutputString, 0, true, false);
      OutputString = "EStop Pulse: " + String(RCPulseData[ESTOPPULSEDATA]);
      DebugOutput(OutputString, 0, true, false);
    }
    /*OutputString = "Left wheel speed: " + String(MotorSpeeds[LEFTSPEED]);
    DebugOutput(OutputString, 0, true, false);
    OutputString = "Left wheel direction: " + String(MotorSpeeds[LEFTDIRECTION]);
    DebugOutput(OutputString, 0, true, false);
    OutputString = "Right wheel speed: " + String(MotorSpeeds[RIGHTSPEED]);
    DebugOutput(OutputString, 0, true, false);
    OutputString = "Right wheel direction: " + String(MotorSpeeds[RIGHTDIRECTION]);
    DebugOutput(OutputString, 0, true, false);
    OutputString = "Switches (A, B, C, D, ESTOP): " + String(Switches[SWITCH_A]) + ", " + String(Switches[SWITCH_B]) + ", " + String(Switches[SWITCH_C]) + ", " + String(Switches[SWITCH_D]) + ", " + String(Switches[SWITCH_ESTOP]);
    DebugOutput(OutputString, 0, true, false);
*/
    return;
  }

  void setup() {
    //Main Setup function
    //serial communication with the motor controller on pins 0/1
    Serial1.begin(38400);
    //serial communication with the console for debug purposes
    Serial.begin(38400);
    //Setting values we will need to repeatedly use for motor control
    MotorSpeeds[LEFTSPEED] = B00000000;      // set Motor 1 speed to 0 to start
    MotorSpeeds[RIGHTSPEED] = B00000000;     // set Motor 2 speed to 0 to start
    MotorSpeeds[LEFTDIRECTION] = B00000001;  // set Motor 1 backwards
    MotorSpeeds[RIGHTDIRECTION] = B00000100; // set Motor 2 forwards
    LeftChecksum = (MotorSpeeds[LEFTDIRECTION] + MotorSpeeds[LEFTSPEED]); // Check other Motor 1 commands against this
    RightChecksum = (MotorSpeeds[RIGHTDIRECTION] + MotorSpeeds[RIGHTSPEED]); // Check other Motor 2 commands against this
    //setup software enable pin
    pinMode(SOFTWAREENABLEPIN, OUTPUT); // set up pin for software enable as an output
    // setup physical switch input pins
    pinMode(SWITCHAPIN, INPUT);
    pinMode(SWITCHBPIN, INPUT);
    pinMode(SWITCHCPIN, INPUT);
    pinMode(SWITCHDPIN, INPUT);
    pinMode(ESTOPPIN, INPUT);
    //Set estop status pin as input
    //Setup RC Input Pins
    pinMode(FWDPULSEPIN, INPUT_PULLUP);
    pinMode(ANGPULSEPIN, INPUT_PULLUP);
    pinMode(ESTOPPULSEPIN, INPUT_PULLUP);
    //Attach High and Low Interrupts to each RC Input
    attachInterrupt(digitalPinToInterrupt(FWDPULSEPIN), FwdPulse, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ANGPULSEPIN), AngPulse, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ESTOPPULSEPIN), EStopPulse, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(FWDPULSEPIN), FwdPulseLow, FALLING);
    //attachInterrupt(digitalPinToInterrupt(ANGPULSEPIN), AngPulseLow, FALLING);
    //attachInterrupt(digitalPinToInterrupt(ESTOPPULSEPIN), EStopPulseLow, FALLING);
    //attach interrupts so the switches are read if any are changed
    attachInterrupt(digitalPinToInterrupt(SWITCHAPIN), ReadSwitches, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SWITCHBPIN), ReadSwitches, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SWITCHCPIN), ReadSwitches, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SWITCHDPIN), ReadSwitches, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ESTOPPIN), ReadSwitches, CHANGE);

    //some serial setup stuff.  take a closer look at the values he's passing in at some point
    // set baud rate to 9600
    Serial1.write(SabertoothAddress);
    Serial1.write(B00001111);
    Serial1.write(B00000010);
    int Checksum0 = (SabertoothAddress + B00000010 + B00001111);
    Serial1.write(Checksum0 & SabertoothMask);

    // set timeout to 200ms
    Serial1.write(SabertoothAddress);
    Serial1.write(B00001110);
    Serial1.write(B00000010);
    Checksum0 = (SabertoothAddress + B00000010 + B00001110);
    Serial1.write(Checksum0 & SabertoothMask);

    //initialize in the EStop state, no action until the physical estop is cycled
    State = STATE_ESTOP;
    NextState = STATE_ESTOP;
    RCEStop = false;
    //Delay 10 seconds to allow for initialization of other components
    //Temp set to 2 seconds for testing
    delay(2000);
    //run all of the various sub-functions and establish an initial run time
    ControlLoop();
    SabertoothMotorCommandLoop();
    ReadSwitches();

    return;
  }

  //main loop
  void loop() {
    //To keep checking for whether the RC EStop is on
    long currentTime = millis();
    RCControl();
    // check to see if enough time has passed to run the control loop
    if ((currentTime - ControlLoopLastTime) >= CONTROLLOOPRATE)
    {
      ControlLoop();
    }

    // check to see if enough time has passed to run motor command loop and if the robot is in
    if ((currentTime - MotorCommandLastSent) >= SABERTOOTHLOOPRATE )
    {
      SabertoothMotorCommandLoop();
    }
    // If switch C is enabled the robot is in debug mode; dump output to console
    if (currentTime - LastDebug >= DEBUGOUTPUTRATE)  //add switch c check
      {
       DebugPrint();
      }

    return;
  }
