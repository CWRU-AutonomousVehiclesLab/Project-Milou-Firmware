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