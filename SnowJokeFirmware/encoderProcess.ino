//!=====================Interupt Callback=====================
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