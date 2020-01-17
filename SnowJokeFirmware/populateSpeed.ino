void rcPopulateSpeed(){
    float ForwardPulse = RCPulseData[FWDPULSEDATA];
    float AngularPulse = RCPulseData[ANGPULSEDATA];
    
    rcLinearSpeed = (abs(ForwardPulse-RCCenter)<=30) ? 0.0 : map(ForwardPulse, RCMin, RCMax, -2.0, 2.0);
    rcAngularSpeed = (abs(AngularPulse-RCCenter)<=30) ? 0.0 :map(AngularPulse, RCMin, RCMax, -2.0, 2.0);
}



void actualObsSpeed(){
    // Get the position first
    long currentLeftPos = leftEncoderPos;
    long currentRightPos = rightEncoderPos;

    // Get time
    long currentTime =  millis();
    
    obsLeftMotorSpeed = ((currentLeftPos-lastLeftPos)*1000.0/(ENCODERTICKPERREV*MotorRevsPerWheelRev*WheelDiameter))/(currentTime-lastEncoderTime);
    obsRightMotorSpeed = ((currentRightPos-lastRightPos)*1000.0/(ENCODERTICKPERREV*MotorRevsPerWheelRev*WheelDiameter))/(currentTime-lastEncoderTime);

    lastLeftPos = currentLeftPos;
    lastRightPos = currentRightPos;
    lastEncoderTime = currentTime;
}
