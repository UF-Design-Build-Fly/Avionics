double channel[6]; //Channels for throttle, aileron, elevator, rudder, and two others
byte throttlePin = 5; //However, this test will only use 1 channel.

void setup() {
  pinMode(throttlePin, INPUT); //Wire connecting signal from throttle channel to pin on board
  Serial.begin(9600);
}

void loop() {

  channel[0] = pulseIn(throttlePin, HIGH); //This setup reads the PWM signal from the throttle channel
  Serial.println(channel[0]);

}
