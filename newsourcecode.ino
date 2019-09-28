//to-do: check the FwdCycle, AngCycle, EStopCycle values in the old code and see if they're necessary for safety

//pins for RC input
#define FWDPULSEPIN 2
#define ANGPULSEPIN 3
#define ESTOPPULSEPIN 4
//positions for pin PWM data in the RCPulseData array
#define FWDPULSEDATA 0
#define ANGPULSEDATA 1
#define ESTOPPULSEDATA 2
//total number of RC channels
#define RCNUMBEROFCHANNELS 3

//holds the temporary start of signal data, for use in timing the PWM width of the RC input signals
uint32_t RCStartPulse[RCNUMBEROFCHANNELS];
//holds data from the RC PWM inputs
uint32_t RCTempPulseData[RCNUMBEROFCHANNELS];
//more permenant pulse data copied over from RCTempPulseData
uint32_t RCTempPulseData[RCNUMBEROFCHANNELS];

//Channel-specific functions to calculate the PWM signal length
//the xxxPulseHigh() functions save the signal start time to the RCStartPulse array
//the xxxPulseLow() functions save the signal length to the RCPulseData array
//this was changed from using a generic timing function for all three in hopes that the time saved by not having to do a DigitalRead() would be greater than the additional time of having three more interrupts

void FwdPulseHigh() {
  RCStartPulse[FWDPULSEDATA] = micros();
}
void FwdPulseLow() {
  RCTempPulseData[FWDPULSEDATA] = micros() - RCStartPulse[FWDPULSEDATA];
}

void AngPulseHigh() {
  RCStartPulse[ANGPULSEDATA] = micros();
}
void AngPulseLow() {
  RCTempPulseData[ANGPULSEDATA] = micros() - RCStartPulse[ANGPULSEDATA];
}

void EStopPulseHigh() {
  RCStartPulse[ESTOPPULSEDATA] = micros();
}
void EStopPulseLow() {
  RCTempPulseData[ESTOPPULSEDATA] = micros() - RCStartPulse[ESTOPPULSEDATA];
}

//Copies over the entire temp array to a more permanant one before accessing so an interrupt won't change values while it's being accessed
void GetRCData() {
  noInterrupts(); //pauses interrupts while reading data
  memcpy((void *)RCPulseData,(const void *)RCTempPulseData,sizeof(RCTempPulseData));  
  interrupts(); //resumes other interrupts
}

void setup() {
  //Main Setup function
  //Setup RC Input Pins
  pinMode(FWDPULSEPIN, INPUT_PULLUP);
  pinMode(ANGPULSEPIN, INPUT_PULLUP);
  pinMode(ESTOPPULSEPIN, INPUT_PULLUP);
  //Attach High and Low Interrupts to each RC Input
  attachInterrupt(digitalPinToIntterupt(FWDPULSEPIN), FwdPulseHigh, RISING);
  attachInterrupt(digitalPinToIntterupt(ANGPULSEPIN), AngPulseHigh, RISING);
  attachInterrupt(digitalPinToIntterupt(ESTOPPULSEPIN), EStopPulseHigh, RISING);
  attachInterrupt(digitalPinToIntterupt(FWDPULSEPIN), FwdPulseLow, FALLING);
  attachInterrupt(digitalPinToIntterupt(ANGPULSEPIN), AngPulseLow, FALLING);
  attachInterrupt(digitalPinToIntterupt(ESTOPPULSEPIN), EStopPulseLow, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:

}

