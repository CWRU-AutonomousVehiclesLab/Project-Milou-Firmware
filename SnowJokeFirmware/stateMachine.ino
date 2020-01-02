//the state changes are all handled in the switch read function, so we use this to take the appropriate action
void ControlLoop()
{
  ControlLoopLastTime = millis(); // time the interval of ControlLoop running
  switch (State) {
    case STATE_ESTOP:
      //set the next state to the switch positions if in hard estop
      if (Switches[SWITCH_ESTOP] == 0) {
        if (Switches[SWITCH_A] == 0) //low - autonomous enable
          NextState = STATE_RC;
        else
          NextState = STATE_AUTONOMOUS;
      }
      break;
    case STATE_RC:
      // Populate RC Speed.
      rcPopulateSpeed();
      break;

    case STATE_AUTONOMOUS:
      autonomousPopulateSpeed();
      break;
  }
  return;
}

void checkStateChange(){
  readSwitches();
}