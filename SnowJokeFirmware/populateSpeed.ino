void rcPopulateSpeed(){
    float ForwardPulse = RCPulseData[FWDPULSEDATA];
    float AngularPulse = RCPulseData[ANGPULSEDATA];
    rcLinearSpeed = map(ForwardPulse, RCMin, RCMax, -1.5, 1.5);
    rcAngularSpeed = map(AngularPulse, RCMin, RCMax, -1.0, 1.0);
}

void autoPopulateSpeed(){
    return;
}
