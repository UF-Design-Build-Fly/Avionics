//The majority of the code initializing the MPU6050 as well as computing the y,p,r angles is sourced from Jeff Rowberg <jeff@rowberg.net>
//and his example code titled, "MPU6050_DMP6_using_DMP_V6.12". This example can be viewed here if you do not already have the MPU6050
//installed on your device: https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6_using_DMP_V6.12/MPU6050_DMP6_using_DMP_V6.12.ino
//Issues with the program freezing? Go here: https://github.com/jrowberg/i2cdevlib/issues/519
//To obtain your own gyro offsets scaled for min sensitivity, use the example code in the MPU6050 library named, "IMU_Zero"
//Here is the link to the file on github: https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/IMU_Zero/IMU_Zero.ino

//How to install I2C and MPU6050 libraries (2 ways)
//1. Download the zip files from here: https://dronebotworkshop.com/mpu-6050-level/
//   There is a step by step process on how to install the files directly into your Arduino IDE
//   You can also watch a video using these libraries here: https://www.youtube.com/watch?v=XCyRXMvVSCw
//2. If you feel comfortable downloading the libraries off of github yourself, you can go to: https://github.com/jrowberg/i2cdevlib

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2021 UF-Design-Build-Fly

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

#include <SD.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"

File myFile;
const int pinCS = 10; //CS terminal is connected to pin 10
const int buttonPin = 6; //Digital pin button is connected to
double yaw, pitch, roll;
bool go = 0; //Used in the "wait" while loop
int buttonState = 0; //When button is pressed buttonState = 1, when button is not pressed buttonState = 0
long startTime = 0;
long Time = 0;
double battery_voltage = 0;

//Used to calculate temperature inside aricraft.
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define RED_LED 7 // (Arduino is 13 (Changed to 7), Teensy is 11, Teensy++ is 6)
#define YELLOW_LED 8 //Standby LED
#define GREEN_LED 9 //Ready to start LED
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  Wire.begin(); //join I2C bus
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(3000, true); //timeout value in uSec
  
  Serial.begin(9600); //Debugging purposes.
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(A0, INPUT); //Used to measure cell voltage
  pinMode(pinCS, OUTPUT);
  digitalWrite(pinCS, HIGH);

//  SD Card initialization
  if (SD.begin()) { //if initialization was successful...
    Serial.println("SD card is ready to use.");   
  }
  else {
    Serial.println("SD card initalization failed");
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, HIGH); //Error Code 111: "SD card initalization failed" (all LED's are turned on).
    digitalWrite(RED_LED, HIGH);
    return;
  }

//  Create/Open file
  myFile = SD.open("ypr.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("yaw,pitch,roll,voltage,temperature,time");
    myFile.close();
    Serial.println("ypr.txt was successfully opened and closed");
  }
  else {
    Serial.println("Error opening ypr.txt");
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, HIGH); //Error code 101: "Error opening ypr.txt" (green and red LED's are turned on).
  }

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("\nPress button to begin DMP programming and demo: "));
  digitalWrite(GREEN_LED, HIGH); //Green light illuminates to show it is ready to start
  
  //"Wait": Button needs to be pushed in order to proceed
  while (go < 1) {
    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH) {
      go = 1;
    }
    delay(100);
    }

  //Moves to standby status
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, HIGH);
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(278);
  mpu.setYGyroOffset(243);
  mpu.setZGyroOffset(68);
  mpu.setXAccelOffset(-2957);
  mpu.setYAccelOffset(-744);
  mpu.setZAccelOffset(1105);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //delay(10000); //Desired amount of time to wait before data is collected;
  digitalWrite(YELLOW_LED, LOW);
  
  Serial.print("Data organized by yaw, pitch, roll, milliseconds (y,p,r,t).");
  Serial.println();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
 
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  if (startTime == 0) {
    startTime = millis();
    //Serial.print(startTime);
  }
  Time = millis();
  Time = Time - startTime;
  
  yaw = ypr[0] * 180 / M_PI;
  pitch = ypr[1] * 180 / M_PI;
  roll = ypr[2] * 180 / M_PI;
  battery_voltage = (analogRead(A0) + 1)/204.8;
  //The raw data from pin A0 represents the voltage level between 0V-5V, however the data
  //currently lies on the linear range 0-1023 (1024 points). To interpret the voltage from the raw 
  //data values, we need to add 1 to shift its values to be between 1-1024, and then divide it by 204.8. 
  //This will give a voltage value between 0.00 V and 5.00 V.

  //The 6 lines of code below are sourced from: https://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/
  //A 10k thermistor and a 10k resistor are used here.
  Vo = analogRead(A1); //A1 is the pin reading the raw voltage values. It is measuring the voltage across the 10k resistor.
  R2 = R1 * (1023.0 / (float)Vo - 1.0); //Magical math
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2)); //Temp in kelvin
  T = T - 273.15; //Degrees Celcius
  T = (T * 9.0)/ 5.0 + 32.0; //Degrees Fahrenheit
  
  //Comma Separated Format
  Serial.print(yaw); //Shown in degrees
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(battery_voltage); //Shown in Volts (V)
  Serial.print(",");
  Serial.print(T); //Shown in degrees Fahrenheit (F)
  Serial.print(",");
  Serial.print(Time); //Shown in milliseconds (ms)
  Serial.println();

  //Data logging values
  myFile = SD.open("ypr.txt", FILE_WRITE);

  if (myFile) {
    
    //Comma Separated Format
    myFile.print(yaw); //Shown in degrees
    myFile.print(",");
    myFile.print(pitch);
    myFile.print(",");
    myFile.print(roll);
    myFile.print(",");
    myFile.print(battery_voltage); //Shown in Volts (V)
    myFile.print(",");
    myFile.print(T); //Shown in degrees Fahrenheit (F)
    myFile.print(",");
    myFile.print(Time); //Shown in milliseconds
    myFile.println();
    myFile.close();
  }
  else {
    Serial.println("Error opening ypr.txt");
  }
  
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(RED_LED, blinkState);
  }
  buttonState = digitalRead(buttonPin);
  if (buttonState == 1) {
    myFile = SD.open("ypr.txt", FILE_WRITE);
    if (myFile) {
      myFile.println("End of Data\n");
      myFile.close();
    }
    else {
      Serial.println("Error opening ypr.txt");
    }
    Serial.println("End of Data");
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    while(1);
  } 
}

//Future additions
//Adding sound devices to indicate certain conditions/states of the avionics package.
