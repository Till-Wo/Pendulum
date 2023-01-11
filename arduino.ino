#include <AFMotor.h>
//This is just a test to see if the setup works

AF_DCMotor motor(1, MOTOR12_64KHZ); 

void setup() {
  Serial.begin(9600);           
  Serial.println("Motor test!");
  
  motor.setSpeed(255);   
}

void loop() {
  Serial.print("tick");
  
  motor.run(FORWARD);      
  delay(400);

  Serial.print("tack");
  motor.run(RELEASE);     
  delay(100);

  
  Serial.print("tock");
  motor.run(BACKWARD);     
  delay(400);
  
  Serial.print("tack");
  motor.run(RELEASE);    
  delay(100);
}
