#include <AFMotor.h>
//This simple script needs to be uploaded to the arduino for the program to work


AF_DCMotor motor(1, MOTOR12_64KHZ); 
int incomingByte;  
void setup() {
  Serial.begin(9600);          
  motor.setSpeed(255);     
}

void loop() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 'F') {
      motor.run(FORWARD);
    }
    if (incomingByte == 'N') {
      motor.run(RELEASE);
    }
    if (incomingByte == 'B') {
       motor.run(BACKWARD);    
    }
  }
}
