//!=====================Interupt Callback=====================
//Channel-specific functions to calculate the PWM signal length
void FwdPulse() {
  if (digitalRead(FWDPULSEPIN)) {
    RCStartPulse[FWDPULSEDATA] = micros();
  }
  else {
    RCTempPulseData[FWDPULSEDATA] = micros() - RCStartPulse[FWDPULSEDATA];
  }
  return;
}

void AngPulse() {
  if (digitalRead(ANGPULSEPIN)) {
    RCStartPulse[ANGPULSEDATA] = micros();
  }
  else {
    RCTempPulseData[ANGPULSEDATA] = micros() - RCStartPulse[ANGPULSEDATA];
  }
  return;
}

void EStopPulse() {
  if (digitalRead(ESTOPPULSEPIN)) {
    RCStartPulse[ESTOPPULSEDATA] = micros();
  }
  else {
    RCTempPulseData[ESTOPPULSEDATA] = micros() - RCStartPulse[ESTOPPULSEDATA];
  }
  //terminalSerial.println("estop pulse off");
  //if EStop Pulse > 1300, disable the software enable output
  if (((int)RCTempPulseData[ESTOPPULSEDATA]) <= EStopThreshold) {
    //DebugOutput("Detected Remote ESTOP Activated!",2);
    State = STATE_ESTOP;
  }
  return;
}

//!=====================Loop Call=====================
//Copies over the entire temp array to a more permanant one before accessing so an interrupt won't change values while it's being accessed
void GetRCData() {
  noInterrupts(); //pauses interrupts while reading data
  memcpy((void *)RCPulseData, (const void *)RCTempPulseData, sizeof(RCTempPulseData));
  interrupts(); //resumes other interrupts
  return;
}
