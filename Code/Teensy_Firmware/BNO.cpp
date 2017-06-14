/* 
 *  This .cpp file defines the content of functions to be called by the main routine
 */

// Allow access to arduino specific datatypes 
#include <arduino.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "imumaths.h"
#include "BNO.h"

// Define variables and constants. Voltatile is used for variables that are only modified in ISRs
// as the compiler is liable to treat them as constants.

// IMU---------------------------------- 

Adafruit_BNO055 bno = Adafruit_BNO055();


// Function output
IMUstruct IMUdat;
//--------------------------------------------------------

// Functions to initialize and utilize IMU comms-------------------------------------------------------------------------------
// communication functions

// initialize IMU
void initIMU(){
  delay(1000);
  Serial.println("Starting IMU");
  if(!bno.begin())
  {
    /* There was a problem  the BNO055 ... check your connections */
    Serial.println("No BNO055 detected");
    IMUdat.err = 1;
  }
  
  bno.setExtCrystalUse(true);
  delay(1000);
  Serial.println("BNO ready");
}

IMUstruct pollIMU(void){
  IMUdat.accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  IMUdat.gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  IMUdat.quat = bno.getQuat();
  IMUdat.mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  IMUdat.eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  IMUdat.lia = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  IMUdat.grv = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  return IMUdat;
}
//-------------------------------------------------------------------------------------------------------------------------------
