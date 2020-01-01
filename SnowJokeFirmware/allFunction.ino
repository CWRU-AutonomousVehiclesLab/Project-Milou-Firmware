
void LeftEncoderPulse() {
  //unlike the RC pulses, encoder pulses are read peak to peak
  EncoderPulseData[LEFTENCODER] = micros() - EncoderStartPulse[LEFTENCODER];
  EncoderStartPulse[LEFTENCODER] = micros();
  return;
}

void RightEncoderPulse() {
  EncoderPulseData[RIGHTENCODER] = micros() - EncoderStartPulse[RIGHTENCODER];
  EncoderStartPulse[RIGHTENCODER] = micros();
  return;
}







void ConvertToMotorSpeeds(uint32_t forwardPercent, uint32_t angularPercent) {
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
  leftPercentage = (forwardPercent - ((WheelSpacing / 2) * angularPercent)) / (1 + (WheelSpacing / 2));
  rightPercentage = (forwardPercent + ((WheelSpacing / 2) * angularPercent)) / (1 + (WheelSpacing / 2));
  //now we have a percentage from -1 to 1 for how fast each wheel should be going

  //calculate the output to the motor controller, which is from -127 to 127
  //first, we'll get a multiplier so we don't command our wheels to go faster than the max desired speed
  //to do that we need to calculate the maximum possible speed of the robot
  maxPossibleSpeed = MaxMotorRPM * (1 / MotorRevsPerWheelRev) * WheelDiameter * PI;
  //now get the speed cap as a percent of the maximum
  //speedCapPercent = MaxDesiredSpeed/maxPossibleSpeed;
  //until we know the constants, just output motor commands at half of max possible speed
  speedCapPercent = 0.5;
  //finally, get the values we're outputting to the motor controller by multiplying these all together
  leftMotorSpeed = leftPercentage * SABERTOOTHMAX * speedCapPercent;
  rightMotorSpeed = leftPercentage * SABERTOOTHMAX * speedCapPercent;
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

  //put into the array for accessing elsewhere
  DesiredSpeeds[LEFTSPEED] = abs(leftMotorSpeed);
  DesiredSpeeds[RIGHTSPEED] = abs(rightMotorSpeed);
  DesiredSpeeds[LEFTDIRECTION] = leftMotorDirection;
  DesiredSpeeds[RIGHTDIRECTION] = rightMotorDirection;

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

  if (Switches[SWITCH_ESTOP] == 0) {
    State = STATE_ESTOP;
    digitalWrite(SOFTWAREENABLEPIN, 0); //output low when in EStop State
  }
  else {
    State = NextState;
    digitalWrite(SOFTWAREENABLEPIN, 1); //output high when not in EStop State
  }
  return;
}
//take the desired speed calculated so far, use PID functions to calculate the actual speed
void PID()
{
  //get time 
  float currentTime = millis();  
  float timeElapsed = currentTime-PIDLastTime;
  //directions shouldn't change at all
  MotorSpeeds[LEFTDIRECTION] = DesiredSpeeds[LEFTDIRECTION];
  MotorSpeeds[RIGHTDIRECTION] = DesiredSpeeds[RIGHTDIRECTION];

  //take the pulse length from the encoder input, and turn it into current speed
  //calculate this by dividing the distance covered in one rotation by the time it takes to complete one rotation, the latter of which is the length of one encoder pulse times the number of pulses per cycle
  float currentSpeedInMetersPerSecond[2];

  currentSpeedInMetersPerSecond[LEFTENCODER] = ((WheelDiameter * PI) / (EncoderPulseData[LEFTENCODER] * ENCODERCYCLES));
  currentSpeedInMetersPerSecond[RIGHTENCODER] = ((WheelDiameter * PI) / (EncoderPulseData[RIGHTENCODER] * ENCODERCYCLES));

  //the final product will be a 0-127 value, just like the desired and final speeds
  int currentSpeeds[2];
  currentSpeeds[LEFTSPEED] = map(currentSpeedInMetersPerSecond[LEFTENCODER], 0, MaxDesiredSpeed, 0, 127);
  currentSpeeds[RIGHTSPEED] = map(currentSpeedInMetersPerSecond[RIGHTENCODER], 0, MaxDesiredSpeed, 0, 127);

  //PID calculations for left wheel
  PIDError[LEFTSPEED] = DesiredSpeeds[LEFTSPEED]-currentSpeeds[LEFTSPEED];
  PIDIntegralError[LEFTSPEED] += PIDError[LEFTSPEED]*timeElapsed;
  PIDDerivativeError[LEFTSPEED] = (PIDError[LEFTSPEED]-PIDPreviousError[LEFTSPEED])/timeElapsed;

  //PID calculations for right wheel
  PIDError[RIGHTSPEED] = DesiredSpeeds[RIGHTSPEED]-currentSpeeds[RIGHTSPEED];
  PIDIntegralError[RIGHTSPEED] += PIDError[RIGHTSPEED]*timeElapsed;
  PIDDerivativeError[RIGHTSPEED] = (PIDError[RIGHTSPEED]-PIDPreviousError[RIGHTSPEED])/timeElapsed;

  //using gain constants, calculate motor speed to output
  MotorSpeeds[LEFTSPEED] = KP*PIDError[LEFTSPEED] + KI*PIDIntegralError[LEFTSPEED] + KD*PIDDerivativeError[LEFTSPEED];
  MotorSpeeds[RIGHTSPEED] = KP*PIDError[RIGHTSPEED] + KI*PIDIntegralError[RIGHTSPEED] + KD*PIDDerivativeError[RIGHTSPEED];
  
  PIDLastTime = currentTime;

}

void SabertoothMotorCommandLoop()
{
  //! TODO - now that the PID is in place Motor Speeds could possibly be given a value higher than 127, impose a cap of some sort
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

  } else if(State==STATE_AUTONOMOUS){       //If running autonomously, call the PID function to get the actual speed we're currently outputting
    PID();   
  } else{
    MotorSpeeds[LEFTSPEED] = DesiredSpeeds[LEFTSPEED];   //no need to do PID for RC mode, so a simple passthrough from desired to output speed
    MotorSpeeds[RIGHTSPEED] = DesiredSpeeds[RIGHTSPEED];
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
  switch (State) {
    case STATE_ESTOP:
      //set the next state to the switch positions if in hard estop
      if (Switches[SWITCH_ESTOP] == 0) {
        if (Switches[SWITCH_A] == 0) //low - autonomous enable
          NextState = STATE_AUTONOMOUS;
        else
          NextState = STATE_RC;
      }
      break;
    case STATE_RC:
    //nothing here now that we moved the RC control to the main loop
    case STATE_AUTONOMOUS:
      //for demoing the PID
      /*
       Desired
       */
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
  
  //The below debugging info is for the purpose of the technical demonstration on 12/2/2019

  //first demonstration: state switching between Autonomous, RC, and EStop
/*
  if(State==STATE_ESTOP)
    OutputString = "State: EStop";
  else if(State==STATE_RC)
    OutputString = "State: RC";
  else if(State==STATE_AUTONOMOUS)
    OutputString = "State: Autonomous";
  DebugOutput(OutputString, 0, true, false);
  OutputString = "Switches (A, B, C, D, ESTOP): " + String(Switches[SWITCH_A]) + ", " + String(Switches[SWITCH_B]) + ", " + String(Switches[SWITCH_C]) + ", " + String(Switches[SWITCH_D]) + ", " + String(Switches[SWITCH_ESTOP]);
  DebugOutput(OutputString, 0, true, false);
 */

  //second demonstration: reading RC signal lengths

  /*
  if (State == STATE_RC) {
    OutputString = "Forward Pulse: " + String(RCPulseData[FWDPULSEDATA]);
    DebugOutput(OutputString, 0, true, false);
    OutputString = "Ang Pulse: " + String(RCPulseData[ANGPULSEDATA]);
    DebugOutput(OutputString, 0, true, false);
    OutputString = "EStop Pulse: " + String(RCPulseData[ESTOPPULSEDATA]);
    DebugOutput(OutputString, 0, true, false);
  }
  */

  //third demonstration: calculating wheel speeds in RC mode

  /*OutputString = "Left wheel speed: " + String(MotorSpeeds[LEFTSPEED] + ", direction: " + String(MotorSpeeds[LEFTDIRECTION]));
    DebugOutput(OutputString, 0, true, false);
    OutputString = "Right wheel speed: " + String(MotorSpeeds[RIGHTSPEED] + ", direction: " + String(MotorSpeeds[RIGHTDIRECTION]));
    DebugOutput(OutputString, 0, true, false);
  */

  //fourth demonstration: getting RPM from encoder input

  
  float EncoderRPM = (60*1000000)/(EncoderPulseData[LEFTENCODER] * 256);
  OutputString = ("Encoder RPM: " + String(EncoderRPM) + "RPM\n");
  DebugOutput(OutputString, 0, true, false);
  
  //fifth demonstration: comparing desired vs. actual speed in autonomous mode
/*
  AutonomousTestAccumulator = AutonomousTestAccumulator + (millis() - AutonomousTestTiming);
  AutonomousTestTiming = millis();
  if(AutonomousTestAccumulator > 5000){
    AutonomousLinearVelocity = 1;
    if(AutonomousTestAccumulator > 10000)
      AutonomousTestAccumulator = 0;
  }else
    AutonomousLinearVelocity = 0;
  OutputString = "Currently giving test speed: " + String(AutonomousLinearVelocity);
  DebugOutput(OutputString, 0, true, false);
  if(State==STATE_AUTONOMOUS){
    OutputString = "Left wheel speed after PID: " + String(MotorSpeeds[LEFTSPEED]) + ", direction: " + String(MotorSpeeds[LEFTDIRECTION]));
    DebugOutput(OutputString, 0, true, false);
  }else{
    OutputString = "Not in autonomous mode";
    DebugOutput(OutputString, 0, true, false);
  }
  */
  return;
}
