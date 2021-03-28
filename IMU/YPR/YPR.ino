//Obtains Yaw, Pitch, and Rolls Angles in degrees.
#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms
  Serial.println(F("Rotation vector enabled"));

}

void loop() {
  if (myIMU.dataAvailable() == true)
  {
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
    Serial.print("ypr\t");
    Serial.print(yawDeg,2);
    Serial.print("\t"); 
    Serial.print(pitchDeg,2);
    Serial.print("\t"); 
    Serial.println(rollDeg,2);
  }
}
