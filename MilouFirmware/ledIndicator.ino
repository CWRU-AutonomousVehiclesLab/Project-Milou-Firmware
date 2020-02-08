// Red: ESTOP; Green: RC, Blue: Autonomous
void writeLEDState(){
    switch (State)
    {
        case STATE_ESTOP:
            digitalWrite(LED_R,HIGH);
            digitalWrite(LED_G,LOW);
            digitalWrite(LED_B,LOW);
            break;

        case STATE_RC:
            digitalWrite(LED_R,LOW);
            digitalWrite(LED_G,HIGH);
            digitalWrite(LED_B,LOW);
            break;

        case STATE_AUTONOMOUS:
            digitalWrite(LED_R,LOW);
            digitalWrite(LED_G,LOW);
            digitalWrite(LED_B,HIGH);            
            break;
    
        default:
            break;
    }
}