//read the values of all switches
void readSwitches()
{
  //! The code here is final. DO not modify. 
  Switches[SWITCH_A] = digitalRead(SWITCHAPIN);
  Switches[SWITCH_B] = digitalRead(SWITCHBPIN);
  Switches[SWITCH_C] = digitalRead(SWITCHCPIN);
  Switches[SWITCH_D] = digitalRead(SWITCHDPIN);
  Switches[SWITCH_ESTOP] = digitalRead(ESTOPPIN);

  // ESTOP heard from the action board
  if (Switches[SWITCH_ESTOP] == 0) {
    DebugOutput("Detected ESTOP Circuit Activated!",2);    
    State = STATE_ESTOP;
  }
  else {    
    State = NextState;
  }

  return;
}