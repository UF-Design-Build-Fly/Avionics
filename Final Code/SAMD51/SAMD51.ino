#include <Servo.h>
#define read_Channel_6 G6
#define read_Channel_7 G7
#define read_Channel_8 G8
#define lights_signal G2
#define winch_signal G3
#define door_signal G4

/*
CH6 (light)
  990 ish for down (dual switch) 
  2011 for up
CH7 (stop, retract, retreive)
  tri state switch
  UP: 990
  Middle: 1500
  Down: 2012  
CH8 (doors)
 tri-state switch
  Up: 990
  Middle: 1500 
  Down: 2012

In testing:
0 sends ~ 0.5 ms signal (HI)
45 sends ~ 1ms signal (HI)
90 sends ~ 1.47ms signal (HI)
135 sends ~ 1.93 ms signal (HI)
180 sends ~ 2.4 ms signal (HI)
*/

byte lights_off = 0;
byte lights_on = 180;

byte winch_release_speed = 90; //Real value either 45 or 135. Need to determine which signals would retrieve and release.
byte winch_retrieve_speed = 90; //Real value either 45 or 135. Also need to determine how fast the servos need to retrieve/release
byte winch_stop = 90;

byte door_closed = 0; //??
byte door_open = 0; //?

float chan6sig, chan7sig, chan8sig = 0;

Servo lights;
Servo winch;
Servo door;

void setup() {
  pinMode(read_Channel_6, INPUT); //Signal sent down tow rope to toggle lights
  pinMode(read_Channel_7, INPUT); //Signal to toggle winch (Back, Stop, Forward)
  pinMode(read_Channel_8, INPUT); //Signal to toggle doors
  lights.attach(lights_signal);
  winch.attach(winch_signal);
  door.attach(door_signal);
  lights.write(lights_off);
  winch.write(winch_stop);
  door.write(door_closed);
}

void loop() {
  chan6sig = pulseIn(read_Channel_6, HIGH);
  chan7sig = pulseIn(read_Channel_7, HIGH);
  chan8sig = pulseIn(read_Channel_8, HIGH);
  gators(chan6sig);
  bassProShop(chan7sig);
  B52(chan8sig);  
}

void gators (float duration) {
  if (duration > 1900) {
    lights.write(lights_on);
  }
  else {
    lights.write(lights_off);
  }
}

void bassProShop (float duration) { //Do something with the winch!
  if (duration < 1100) {
    winch.write(winch_release_speed);
  }
  else if (duration > 1900) {
    winch.write(winch_retrieve_speed);
  }
  else {
    winch.write(winch_stop);
  }
}

void B52 (float duration) { //Open the hatch!...or close...
  if (duration < 1250) { //If tri-state switch is in 
    door.write(door_open);
  }
  else {
    door.write(door_closed);
  }
}
