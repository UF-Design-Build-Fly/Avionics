## What needs to be done?
- 1 switch (3 positions) controls the distance the winch/sensor is released: Release, Stop, and Retract
- Hall sensor counts revolutions (After a certain # the air brakes and lights condition on sensor become true/on)
- Bomb Bay opens automatically and closes only after a certain number of revolutions (sensor payload is clear of aircraft). We want to optimize the door being open for the least amount of time.
- When retracting, bomb bay opens again (pnce sensor is close enough to the aircraft) when rotation count is significantly less than full depolyment count. Air brakes and lights condition are false/off.
- Bomb bay door closes after sensor is reeled in completely.
- Signal controlling sensor payload technically handles one condition, ON/OFF. If ON, lights and air brakes are true. Else, lights and air bakes are false.
- May have to use interrupts in software to give person flying aircraft immediate control when flipping switch if stuck in a loop (depends on how fast code is executed).
- Sensors? Data logging? (not priority)

### Notes
CH6 (light)
  dual switch 
	990 ish for down (lights off)
	2011 for up (lights on)
CH7 (stop, retract, retreive)
  tri state switch
	UP: 990 (release)
	Middle: 1500 (stop)
	Down: 2012 (retrieve)	
CH8 (doors)
  tri-state switch
	Up: 990 (open)
	Middle: 1500 (closed)
	Down: 2012 (closed)


