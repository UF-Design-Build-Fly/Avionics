//Servo library is used to output "PWM" signals
#include <Servo.h>
double channel[3]; //Possible channels we could be reading from.
bool state = 0; //State keeps track when and when not to use attach() and detach().

//G0 reads channel
//G1 outputs signal

Servo myServo;

void setup() {
  pinMode(G0, INPUT);
}

void loop() {

  channel[0] = pulseIn(G0, HIGH); //Reads PWM from receiver channel on G0 pin
  if (channel[0] < 1250){ //If PWM signal from channel is less than 1250 microseconds, run code
    
    if (state == 0) { //Condition to output signal on G1 pin
      myServo.attach(G1);
   }
   
   myServo.write(40); //In testing, 40 degree angle resulted in 1ms signal. 
   state = 1;
  }
  else {
    
    if (state == 1) { //Condition to stop outputting signals
      myServo.detach();
      state = 0; //
   }
  }
}
