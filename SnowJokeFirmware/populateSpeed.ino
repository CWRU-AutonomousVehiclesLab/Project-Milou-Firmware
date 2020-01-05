void rcPopulateSpeed(){
    float ForwardPulse = RCPulseData[FWDPULSEDATA];
    float AngularPulse = RCPulseData[ANGPULSEDATA];
    
    rcLinearSpeed = (abs(ForwardPulse-RCCenter)<=100) ? 0.0 : map(ForwardPulse, RCMin, RCMax, -1.5, 1.5);
    rcAngularSpeed = (abs(AngularPulse-RCCenter)<=100) ? 0.0 :map(AngularPulse, RCMin, RCMax, -1.0, 1.0);
}

void autoPopulateSpeed(){
    return;
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
