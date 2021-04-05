#include <SD.h>
#include <SPI.h>

/*
Pinout on NANO
Connect the 5V pin to the 5V pin on the Arduino
Connect the GND pin to the GND pin on the Arduino
Connect CLK to pin 13 
Connect DO(MISO) to pin 12 
Connect DI(MOSI) to pin 11 
Connect CS to pin 10
*/

/*
Pinout on SAMD51
Connect the Vcc pin to the 3V3 pin on the Carrier Board
Connect the GND pin to the GND pin on the Carrier Board
Connect SCK to SCK
Connect DO to CIPO
Connect DI to COPI
Connect CS to CS
*/

File myFile;

void setup() {

  pinMode(CS, OUTPUT); //Unsure if this line is required or not 
  Serial.begin(9600);
  while(!Serial);
  
  //  SD Card initialization
  if (SD.begin()) { //if initialization was successful...
    Serial.println("SD card is ready to use.");   
  }
  else {
    Serial.println("SD card initalization failed");
    return;
  }

//  Create/Open file
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("Hello World");
    myFile.close();
    Serial.println("test.txt was successfully opened and closed");
  }
  else {
    Serial.println("Error opening test.txt");
   
    return;
  }
}

void loop() {
  
}
