//the state changes are all handled in the switch read function, so we use this to take the appropriate action
void ControlLoop()
{
  ControlLoopLastTime = millis(); // time the interval of ControlLoop running
  writeLEDState(); //Indicate state
  switch (State) {
    case STATE_ESTOP:
      activateESTOP();
      //! set the next state to the switch positions if in hard estop
      if (Switches[SWITCH_ESTOP] == 0) {
        if (Switches[SWITCH_D] == 0)
          NextState = STATE_RC;
        else
          NextState = STATE_AUTONOMOUS;
      }
      ROS_FEEDBACKESTOP();
      break;

    case STATE_RC:
      //! 0. Enable Sabertooth
      enableSabertooth();
      //! 1. Update the Command Velocity and 
      rcPopulateSpeed(); //This should write rcLinearSpeed, rcAngularSpeed
      //! 2. calculate what that mean in Each wheel speeds
      ik(rcLinearSpeed,rcAngularSpeed); //This should populate desLeftMotorSpeed,DesRightMotorSpeed
      //! 3. PID smooth the desired left motor speed and right motor speed.
      actualObsSpeed();
      PID(); //This should populate cmdLeftMotorSpeed,cmdRightMotorSpeed
      //! 4. eventually the command generated above will be observed by sabertooth writer and will write the robot.

      break;

    case STATE_AUTONOMOUS:
      //! 0. Enable Sabertooth
      enableSabertooth();    
      //! 1. Update the Command Velocity and 
      autoPopulateSpeed();
      //! 2. calculate what that mean in Each wheel speeds
      ik(autoLinearVelocity,autoAngularVelocity); //This should populate desLeftMotorSpeed,DesRightMotorSpeed
      //! 3. PID smooth the desired left motor speed and right motor speed.
      actualObsSpeed();
      PID(); //This should populate cmdLeftMotorSpeed,cmdRightMotorSpeed
      //! 4. eventually the command generated above will be observed by sabertooth writer and will write the robot.      
      break;
  }
  return;
}

void startupCheck(){
  //!====================State initialization====================
  //initialize in the EStop state, no action until the physical estop is cycled  
  State = STATE_ESTOP;
  NextState = STATE_ESTOP;
  GetRCData();
  readSwitches();  
  sabertoothInit();
  ROSPublish();
  digitalWrite(ROSENABLEPIN, HIGH); // All set indicator, enable motor
  activateESTOP();
  ControlLoop();
}