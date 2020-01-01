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