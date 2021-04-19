#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <SparkFun_BNO080_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
#include <SparkFun_LPS25HB_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_LPS25HB
#include <Adafruit_MCP9808.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS

#define read_Channel_1 G0
#define read_Channel_2 G1
#define read_Channel_3 G2
#define read_Channel_4 G3
#define read_Channel_6 G4
#define read_Channel_7 G5
#define read_Channel_8 G6

#define LEDpin D0
#define winch_signal G11//63 on ATP, 43 or 38 on SAMD is G10
#define door_signal G9

#define greenLED G7
#define yellowLED G8
#define redLED D1

byte winch_release_speed = 60; //Real value either 45 or 135. Need to determine which signals would retrieve and release.
byte winch_retrieve_speed = 115; //Real value either 45 or 135. Also need to determine how fast the servos need to retrieve/release.
byte winch_stop = 93;

byte door_closed = 40; //0 = 543us 45 = 1ms
byte door_open = 140; //180 = 2.4ms, 135 = 1.935

float chan1sig, chan2sig, chan3sig, chan4sig, chan6sig, chan7sig, chan8sig = 0;

Servo winch;
Servo door;
SFE_UBLOX_GNSS myGNSS;
LPS25HB pressureSensor;
BNO080 myIMU;
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

File IMU_File;
File Pressure_File;
File Ada_File;
File GPS_File;
File PWM_File;

int IMU_update = 50; //50 ms update;
long time_taken_to_initialize = 0;
long GPS_lastTime = 0;
long ada_lastTime = 0;
long wait_time = 60000; //Wait 1 minute before taking readings from sensors
int transmitter_state = 0;

void setup() {
  //Serial.begin(9600);
  //while(!Serial);
  initialize_Pins_and_Servos();
  initialize_Micro_SD();
  initialize_Sensors();
  WAIT();
}

void loop() {
  IMU();
  PWM();
  pressure_and_temp();
  ada_Temperature();
  GPS();
}

void IMU() {
  if (myIMU.dataAvailable() == true)
  {
    /*
    float Ax = myIMU.getAccelX();
    float Ay = myIMU.getAccelY();
    float Az = myIMU.getAccelZ();
    float gyroX = myIMU.getGyroX();
    float gyroY = myIMU.getGyroY();
    float gyroZ = myIMU.getGyroZ();
    */
    float quatI = myIMU.getQuatI();
    float quatJ = myIMU.getQuatJ();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    double JJ = quatJ * quatJ; // 2 Uses below
    double roll = atan2(2 * (quatReal * quatI + quatJ * quatK), 1 - 2*(quatI * quatI + JJ));
    double pitch = -asin(2 * quatReal * quatJ - quatI * quatK);
    double yaw = -atan2(2 * (quatReal * quatK + quatI * quatJ), 1 - 2*(JJ+quatK * quatK));
    float rollDeg  = 57.2958 * roll;
    float pitchDeg = 57.2958 * pitch;
    float yawDeg   = 57.2958 * yaw;

    IMU_File = SD.open("IMU.txt", FILE_WRITE);
    if (IMU_File) {
      IMU_File.print(yawDeg,2);
      IMU_File.print(","); 
      IMU_File.print(pitchDeg,2);
      IMU_File.print(","); 
      IMU_File.print(rollDeg,2);
      IMU_File.print(",");
      IMU_File.print(quatI,2);
      IMU_File.print(","); 
      IMU_File.print(quatJ,2);
      IMU_File.print(","); 
      IMU_File.print(quatK,2);
      IMU_File.print(",");
      IMU_File.print(quatReal,2);
      IMU_File.print(",");
      /*
      IMU_File.print(Ax,2);
      IMU_File.print(","); 
      IMU_File.print(Ay,2);
      IMU_File.print(",");
      IMU_File.print(Az,2);
      IMU_File.print(",");
      IMU_File.print(gyroX,2);
      IMU_File.print(","); 
      IMU_File.print(gyroY,2);
      IMU_File.print(",");
      IMU_File.print(gyroZ,2);
      IMU_File.print(",");
      */
      IMU_File.println(millis());
      IMU_File.close();
    }
    Serial.println("IMU");
  }
}

void PWM() {
  chan1sig = pulseIn(read_Channel_1, HIGH);
  chan2sig = pulseIn(read_Channel_2, HIGH);
  chan3sig = pulseIn(read_Channel_3, HIGH);
  chan4sig = pulseIn(read_Channel_4, HIGH);
  chan6sig = pulseIn(read_Channel_6, HIGH);
  chan7sig = pulseIn(read_Channel_7, HIGH);
  chan8sig = pulseIn(read_Channel_8, HIGH);
  gators(chan6sig);
  bassProShop(chan7sig);
  B52(chan8sig);

  PWM_File = SD.open("PWM.txt", FILE_WRITE);
  if (PWM_File) {
    PWM_File.print(chan1sig);
    PWM_File.print(",");
    PWM_File.print(chan2sig);
    PWM_File.print(",");
    PWM_File.print(chan3sig);
    PWM_File.print(",");
    PWM_File.print(chan4sig);
    PWM_File.print(",");
    PWM_File.print(chan6sig);
    PWM_File.print(",");
    PWM_File.print(chan7sig);
    PWM_File.print(",");
    PWM_File.print(chan8sig);
    PWM_File.print(",");
    PWM_File.println(millis());
    PWM_File.close();
  }
  Serial.println("PWM");
}

void pressure_and_temp () {
  
  float pressure = pressureSensor.getPressure_hPa();
  float temperature_C = pressureSensor.getTemperature_degC();
  
  Pressure_File = SD.open("Pressure.txt", FILE_WRITE);
  if (Pressure_File) {
    Pressure_File.print(pressure);
    Pressure_File.print(",");
    Pressure_File.print(temperature_C);
    Pressure_File.print(",");
    Pressure_File.println(millis());
    Pressure_File.close();
  }
  Serial.println("Pressure");
}

void ada_Temperature () {
  if (millis() - ada_lastTime > 500)
  {
    ada_lastTime = millis(); //Update the timer
    tempsensor.wake();
    float c = tempsensor.readTempC();
    float f = tempsensor.readTempF();
    tempsensor.shutdown_wake(1);
    
    Ada_File = SD.open("AdaTemp.txt", FILE_WRITE);
    if (Ada_File) {
      Ada_File.print(c, 4);
      Ada_File.print(",");
      Ada_File.print(f, 4);
      Ada_File.print(",");
      Ada_File.println(millis());
      Ada_File.close();
    }
    Serial.println("AdaTemp");
  }
}

void GPS()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - GPS_lastTime > 1000)
  {
    GPS_lastTime = millis(); //Update the timer
    
    long latitude = myGNSS.getLatitude();
    long longitude = myGNSS.getLongitude();
    long altitude = myGNSS.getAltitude();
    long speed = myGNSS.getGroundSpeed();
    long heading = myGNSS.getHeading();
    byte SIV = myGNSS.getSIV();

    GPS_File = SD.open("GPS.txt", FILE_WRITE);
    if (GPS_File) {
      GPS_File.print(latitude);
      GPS_File.print(",");
      GPS_File.print(longitude);
      GPS_File.print(",");
      GPS_File.print(altitude);
      GPS_File.print(",");
      GPS_File.print(speed);
      GPS_File.print(",");
      GPS_File.print(heading);
      GPS_File.print(",");
      GPS_File.print(SIV);
      GPS_File.print(",");
      GPS_File.println(millis());
      GPS_File.close();
    }
    Serial.println("GPS");
  }
}

void gators (float duration) {
  if (duration > 1900) {
    digitalWrite(LEDpin, HIGH); //Turns LEDs on
  }
  else {
    digitalWrite(LEDpin, LOW); //Turns/keeps LEDs off
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

void initialize_Micro_SD () {
/*
Pinout on SAMD51
Connect the Vcc pin to the 3V3 pin on the Carrier Board
Connect the GND pin to the GND pin on the Carrier Board
Connect SCK to SCK
Connect DO to CIPO
Connect DI to COPI
Connect CS to D1
*/
  pinMode(CS, OUTPUT); //D1 is used as chip select
  digitalWrite(CS, HIGH); //avoids chip select contention

//  SD Card initialization
  if (SD.begin(CS)) { //if initialization was successful...
     Serial.println("SD card is ready to use.");   
  }
  else {
    digitalWrite(yellowLED, HIGH); //Error Code 111
    digitalWrite(greenLED, HIGH);
    Serial.println("SD card initalization failed");
    return;
  }

  IMU_File = SD.open("IMU.txt", FILE_WRITE);
  if (IMU_File) {
    IMU_File.println("yaw, pitch, roll, qI, qJ, qK, qW, time\t");
    IMU_File.close();
    Serial.println("IMU.txt was successfully opened and closed");
  }
  else {
   digitalWrite(yellowLED, HIGH); //Error Code 111
   digitalWrite(greenLED, HIGH);
   Serial.println("Error opening IMU.txt");
    return;
  }

  Pressure_File = SD.open("Pressure.txt", FILE_WRITE);
  if (Pressure_File) {
    Pressure_File.println("hPa, degC, time\t");
    Pressure_File.close();
    Serial.println("Pressure.txt was successfully opened and closed");
  }
  else {
    digitalWrite(yellowLED, HIGH); //Error Code 111
    digitalWrite(greenLED, HIGH);
    Serial.println("Error opening Pressure.txt");
    return;
  }

  Ada_File = SD.open("AdaTemp.txt", FILE_WRITE);
  if (Ada_File) {
    Ada_File.println("C, F");
    Ada_File.close();
    Serial.println("AdaTemp.txt was successfully opened and closed");
  }
  else {
    digitalWrite(yellowLED, HIGH); //Error Code 111
    digitalWrite(greenLED, HIGH);
    Serial.println("Error opening AdaTemp.txt");
    return;
  }

  GPS_File = SD.open("GPS.txt", FILE_WRITE);
  if (GPS_File) {
    GPS_File.println(" (degrees * 10^-7), (degrees * 10^-7), (mm), (mm/s), (degrees * 10^-5), SIV, time");
    GPS_File.close();
    Serial.println("GPS.txt was successfully opened and closed");
  }
  else {
    digitalWrite(yellowLED, HIGH); //Error Code 111
    digitalWrite(greenLED, HIGH);
    Serial.println("Error opening GPS.txt");
    return;
  }

  PWM_File = SD.open("PWM.txt", FILE_WRITE);
  if (PWM_File) {
    PWM_File.println("Channel: 1, 2, 3, 4, 6, 7, 8, time");
    PWM_File.close();
    Serial.println("PWM.txt was successfully opened and closed");
  }
  else {
    digitalWrite(yellowLED, HIGH); //Error Code 111
    digitalWrite(greenLED, HIGH);
    Serial.println("Error opening PWM.txt");
    return;
  }
}

void initialize_Pins_and_Servos () {
  pinMode(read_Channel_1, INPUT);
  pinMode(read_Channel_2, INPUT);
  pinMode(read_Channel_3, INPUT);
  pinMode(read_Channel_4, INPUT);
  pinMode(read_Channel_6, INPUT); //Signal sent down tow rope to toggle lights
  pinMode(read_Channel_7, INPUT); //Signal to toggle winch (Back, Stop, Forward)
  pinMode(read_Channel_8, INPUT); //Signal to toggle doors
  
  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(LEDpin, OUTPUT);
  winch.attach(winch_signal);
  door.attach(door_signal);
  
  digitalWrite(LEDpin, LOW);
  winch.write(winch_stop);
  door.write(door_closed);
  digitalWrite(redLED, HIGH);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);
}

void initialize_Sensors () {
  Wire.begin();
  Serial.println("Wire begin");

  if (myIMU.begin() == false)
  {
    digitalWrite(yellowLED, HIGH);
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  else {
    Serial.println("IMU is good");
  }

  pressureSensor.begin(); // Begin links an I2C port and I2C address to the sensor, sets an I2C speed, begins I2C on the main board, and then sets default settings

  if (pressureSensor.isConnected() == false) // The library supports some different error codes such as "DISCONNECTED"
  {
    digitalWrite(yellowLED, HIGH);
    Serial.println("LPS25HB disconnected. Reset the board to try again.");     // Alert the user that the device cannot be reached
    Serial.println("Are you using the right Wire port and I2C address?");      // Suggest possible fixes
    Serial.println("See Example2_I2C_Configuration for how to change these."); // Suggest possible fixes
    Serial.println("");
    while (1);
  }
  else {
    Serial.println("Pressure Sensor is good");
  }

  if (!tempsensor.begin(0x18)) {
    digitalWrite(yellowLED, HIGH);
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    while (1);
  }
  else {
    Serial.println("AdaFruit Temperature Sensor is good");
  }

  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
 
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    digitalWrite(yellowLED, HIGH);
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  else {
    Serial.print("GPS is good");
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  Serial.println("Wire clock set");

  myIMU.enableRotationVector(IMU_update);
}

void WAIT () {
  Serial.println("EVERYTHING INITIALIZED");

  while(transmitter_state == 0) {
    chan6sig = pulseIn(read_Channel_6, HIGH);
    chan7sig = pulseIn(read_Channel_7, HIGH);
    chan8sig = pulseIn(read_Channel_8, HIGH);

    Serial.print(chan6sig);
    Serial.print(",");
    Serial.print(chan7sig);
    Serial.print(",");
    Serial.print(chan8sig);

    if (chan6sig < 1900 && chan6sig > 100 && chan7sig < 1600 && chan7sig > 1400 && chan8sig > 1100) {
      transmitter_state = 1; //Switches are now in our defined neutral position
      Serial.print(",");
      Serial.println(transmitter_state);
    }
    else {
      Serial.print(",");
      Serial.println(transmitter_state);
    }
  }
  
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, HIGH);
  
  time_taken_to_initialize = millis();

  while (millis() - time_taken_to_initialize < wait_time){
    chan6sig = pulseIn(read_Channel_6, HIGH);
    chan7sig = pulseIn(read_Channel_7, HIGH);
    chan8sig = pulseIn(read_Channel_8, HIGH);
    gators(chan6sig);
    bassProShop(chan7sig);
    B52(chan8sig);
    Serial.println("Waiting");
  }
  
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, HIGH);
}
