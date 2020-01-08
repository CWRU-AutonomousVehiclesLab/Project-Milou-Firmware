// Referring to : https://playground.arduino.cc/Main/RotaryEncoders/
// Interrupt Example (the Encoder interrupts the processor). Uses both External Interrupt pins, simplifies the interrupt service routines of the above.


//!=====================Interupt Callback=====================
void LeftEncoderA(){
  // Test transition
  leftAset = digitalRead(LEFTENCODER_A)== HIGH;
  // adn adjust counter + if A leads B
  leftEncoderPos += (leftAset != leftBset) ? -1 : +1;
}

void LeftEncoderB() {
  // Test transition
  leftBset = digitalRead(LEFTENCODER_B) == HIGH;
  // and adjust counter + if B follows A
  leftEncoderPos += (leftAset == leftBset) ? -1 : +1;
}


void RightEncoderA(){
  // Test transition
  rightAset = digitalRead(RIGHTENCODER_A)== HIGH;
  // adn adjust counter + if A leads B
  rightEncoderPos += (rightAset != rightBset) ? -1 : +1;
}

void RightEncoderB() {
  // Test transition
  rightBset = digitalRead(RIGHTENCODER_B) == HIGH;
  // and adjust counter + if B follows A
  rightEncoderPos += (rightAset == rightBset) ? -1 : +1;
}

/*
Basically, if the pin that changed now matches the other pin, it's lagging behind it, and if the pin that changed is now different, then it's leading it.
*/
