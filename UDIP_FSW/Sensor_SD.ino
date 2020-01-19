#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h> 

#include <SD.h>
File myFile;
#define vSweepPin DAC0

// i2c comms
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

void setupSensor(){
  //accel range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);

  //mag range
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);

  //gyro range
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}
void setup() {
  Serial.begin(9600);
  while(!Serial){
    ;
  }
  // put your setup code here, to run once:
  //SD card setup
  Serial.println("Initiallizing SD Card...");
  if(!SD.begin(10)){
    Serial.println("initialization failed!");
    while(1);
  }
   Serial.println("initialization done.");
  myFile = SD.open("test.csv",FILE_WRITE);
  if(myFile){
    Serial.println("File Opened");
    myFile.println("Time,X,Y,Z");
    myFile.close();
    Serial.println("done.");
  }
  else{
    Serial.println("Some Error with file");
  }
  if(!lsm.begin()){
    Serial.println("Not detecting LSM9DS0");

}
 // analogWriteResolution(12);
 for(int i=0; i<50; i++){
  Serial.println("Loop is running");
  myFile=SD.open("test.csv",FILE_WRITE);
  lsm.read();
  myFile.print(millis());
  myFile.print(",");
  myFile.print((int)lsm.accelData.x);
  myFile.print(",");
  myFile.print((int)lsm.accelData.y);
  myFile.print(",");
  myFile.print((int)lsm.accelData.z);
  myFile.print("\n");
  //myFile.close();
  delay(250);
 }
 
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  Serial.println("Loop is running");
  myFile=SD.open("Accel_Data",FILE_WRITE);
  lsm.read();
  myFile.print(millis());
  myFile.print(",");
  myFile.print((int)lsm.accelData.x);
  myFile.print(",");
  myFile.print((int)lsm.accelData.y);
  myFile.print(",");
  myFile.print((int)lsm.accelData.z);
  myFile.print("\n");
  myFile.close();
  delay(250); 
  */
}
