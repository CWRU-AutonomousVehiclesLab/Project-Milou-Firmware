void rcPopulateSpeed(){
    int ForwardPulse = RCPulseData[FWDPULSEDATA];
    int AngularPulse = RCPulseData[ANGPULSEDATA];
    rcLinearSpeed = map(ForwardPulse, RCMin, RCMax, -2.0, 2.0);
    rcAngularSpeed = map(AngularPulse, RCMin, RCMax, -1.0, 1.0);
}

void autoPopulateSpeed(){
    return;
}
