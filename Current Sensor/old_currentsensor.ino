//References: https://www.digikey.com/en/products/detail/SEN0098/1738-1102-ND/6588524?itemSeq=352903608

// Initalize SD Card read/write libraries  
#include <SPI.h>  //Serial Peripheral Interface
#include <SD.h>  //SD Card

File myFile;

// Initalize Pin for Current Sensor
const int currentSensor = A0;                         //may need to change

// Define All Constants
const float VCC = 3.3;                                //supply voltage 5V or 3.3V
const float quiescent_Vout = 0.12;                    //from datasheet for unidirectional 50A sensor. units: V
float QOV = quiescent_Vout*VCC;                       //set QOV
const float sensitivity = 60;                         //from datasheet for unidirectional 50A, PFF sensor. units: mV/A
float FACTOR = sensitivity/1000;                      //set sensitivity FACTOR
const float cutOffLimit = 1.00;                       //reading cut off current 1.00 is 1 Ampere
float cutOff = FACTOR/cutOffLimit;                    //converts current cut off to mV
const float zeroCalibrationVal = 0.007;               //depends on what we need

void setup() {
serial begin(9600);
myFile = SD.open("test.txt", FILE_WRITE);

serial.println("Current Sensor");
}

void loop() {
  
//Calculate & Record Current Value
  int sensorVal = analogRead(currentSensor);          //Read the voltage form sensor
  float voltage_raw = (sensorVal/1024.0) * 5.0;
  voltage = voltage_raw - QOV + zeroCalibrationVal;  
  float current = voltage / FACTOR;
  myFile.println("V: ");
  myFile.println(voltage, 3);
  myFile.println("V, I: ");
  myFile.println(current, 2); myFile.println("A");
  delay(500);
}


    
   
  
  
 
