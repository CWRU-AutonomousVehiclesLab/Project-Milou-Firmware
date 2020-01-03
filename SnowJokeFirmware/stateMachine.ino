//the state changes are all handled in the switch read function, so we use this to take the appropriate action
void ControlLoop()
{
  ControlLoopLastTime = millis(); // time the interval of ControlLoop running
  writeLEDState(); //Indicate state
  switch (State) {
    case STATE_ESTOP:
      //! set the next state to the switch positions if in hard estop
      activateESTOP();
      if (Switches[SWITCH_ESTOP] == 0) {
        if (Switches[SWITCH_A] == 0)
          NextState = STATE_RC;
        else
          NextState = STATE_AUTONOMOUS;
      }
      break;

    case STATE_RC:
      //! 1. Update the Command Velocity and 
      rcPopulateSpeed(); //This should write rcLinearSpeed, rcAngularSpeed
      //! 2. calculate what that mean in Each wheel speeds
      ik(rcLinearSpeed,rcAngularSpeed); //This should populate desLeftMotorSpeed,DesRightMotorSpeed
      //! 3. PID smooth the desired left motor speed and right motor speed.
      PID(); //This should populate cmdLeftMotorSpeed,cmdRightMotorSpeed
      //! 4. eventually the command generated above will be observed by sabertooth writer and will write the robot.

      break;

    case STATE_AUTONOMOUS:
      autoPopulateSpeed();
      break;
  }
  return;
}

void checkStateChange(){
  readSwitches();
}