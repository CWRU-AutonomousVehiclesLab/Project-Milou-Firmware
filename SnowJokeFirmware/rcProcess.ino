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
    //terminalSerial.println("estop pulse off");
    //if EStop Pulse > 1300, disable the software enable output
    if (RCTempPulseData[ESTOPPULSEDATA] > EStopThreshold) {
      State = STATE_ESTOP;
      NextState = STATE_ESTOP;
      digitalWrite(SOFTWAREENABLEPIN, 1); //output low when in EStop State
      RCEStop = true;
      DesiredSpeeds[LEFTSPEED] = 0;
      DesiredSpeeds[RIGHTSPEED] = 0;
      DesiredSpeeds[LEFTDIRECTION] = 0;
      DesiredSpeeds[RIGHTDIRECTION] = 0;
    }
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

void RCControl() {
  GetRCData();
  uint32_t ForwardPulse = RCPulseData[FWDPULSEDATA];
  uint32_t AngularPulse = RCPulseData[ANGPULSEDATA];
  uint32_t EStopPulse = RCPulseData[ESTOPPULSEDATA];

  //if EStop Pulse > 1300, disable the software enable output
  if (EStopPulse > EStopThreshold) {
    State = STATE_ESTOP;
    NextState = STATE_ESTOP;
    digitalWrite(SOFTWAREENABLEPIN, 1); //output low when in EStop State
    activateESTOP();
    return;
  }
  else {
    if (State == STATE_RC) {            //use the RC values to output if in RC State
      forwardPercent = map(ForwardPulse, RCMin, RCMax, -1, 1);
      angularPercent = map(AngularPulse, RCMin, RCMax, -1, 1);
    }
  }
  return;
}