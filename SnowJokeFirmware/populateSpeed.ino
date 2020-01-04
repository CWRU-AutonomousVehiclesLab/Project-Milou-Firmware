void rcPopulateSpeed(){
    float ForwardPulse = RCPulseData[FWDPULSEDATA];
    float AngularPulse = RCPulseData[ANGPULSEDATA];
    rcLinearSpeed = map(ForwardPulse, RCMin, RCMax, -1.5, 1.5);
    rcAngularSpeed = map(AngularPulse, RCMin, RCMax, -1.0, 1.0);
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
    
    obsLeftMotorSpeed = ((currentLeftPos-lastLeftPos)*1000.0/1024.0)/(currentTime-lastEncoderTime);
    obsRightMotorSpeed = ((currentRightPos-lastRightPos)*1000.0/1024.0)/(currentTime-lastEncoderTime);

    lastLeftPos = currentLeftPos;
    lastRightPos = currentRightPos;
    lastEncoderTime = currentTime;
}
