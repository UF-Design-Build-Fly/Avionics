#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  myIMU.begin();

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableGyro(50); //Send data update every 50ms
  myIMU.enableAccelerometer(50); //Send data update every 50ms
  myIMU.enableMagnetometer(50); //Send data update every 50ms

  Serial.println(F("Gyro enabled"));
  Serial.println(F("Output in form x, y, z, in radians per second"));
  Serial.println(F("Accelerometer enabled"));
  Serial.println(F("Output in form x, y, z, in m/s^2"));
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float Gx = myIMU.getGyroX();
    float Gy = myIMU.getGyroY();
    float Gz = myIMU.getGyroZ();
    float Ax = myIMU.getAccelX();
    float Ay = myIMU.getAccelY();
    float Az = myIMU.getAccelZ();
    float Mx = myIMU.getMagX();
    float My = myIMU.getMagY();
    float Mz = myIMU.getMagZ();
    byte accuracy = myIMU.getMagAccuracy();

    //Gyro
    Serial.print(Gx, 2);
    Serial.print(F(","));
    Serial.print(Gy, 2);
    Serial.print(F(","));
    Serial.print(Gz, 2);
    Serial.print(F(","));
    
    //Accel
    Serial.print(Ax, 2);
    Serial.print(F(","));
    Serial.print(Ay, 2);
    Serial.print(F(","));
    Serial.print(Az, 2);
    Serial.print(F(","));

    //Mag
    Serial.print(Mx, 2);
    Serial.print(F(","));
    Serial.print(My, 2);
    Serial.print(F(","));
    Serial.print(Mz, 2);
    Serial.print(F(","));
    printAccuracyLevel(accuracy);
    Serial.print(F(","));

    Serial.println();
  }
}

//Given a accuracy number, print what it means
void printAccuracyLevel(byte accuracyNumber)
{
  if(accuracyNumber == 0) Serial.print(F("Unreliable"));
  else if(accuracyNumber == 1) Serial.print(F("Low"));
  else if(accuracyNumber == 2) Serial.print(F("Medium"));
  else if(accuracyNumber == 3) Serial.print(F("High"));
}
