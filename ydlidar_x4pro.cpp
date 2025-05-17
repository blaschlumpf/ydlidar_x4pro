/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR Arduino 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.eaibot.com
 * 
 * https://github.com/EAIBOT/ydlidar_arduino/tree/master
 *
 * modified V/2025 by Florian Schäffer, https://www.blafusel.de
 *
 */
 
#include "YDLidar.h"

YDLidar lidar;      // You need to create an driver instance 

bool isScanning = false;   

#define YDLIDAR_MOTOR_SCTP 3      // The PWM pin for control the speed of YDLIDAR's motor. 
#define YDLIDAR_BAURATE 128000        // LIDAR Baudrate
#define YDLIDAR_MINDIST 120         // min Distance in mm
#define YDLIDAR_MAXDIST 1000
#define YDLIDAR_M_CTR_VOLT 2.15      // Motor speed control voltage

void setup() {
  lidar.begin(Serial, YDLIDAR_BAURATE);     // bind the YDLIDAR driver to the arduino hardware serial
  pinMode(YDLIDAR_MOTOR_SCTP, OUTPUT);      // PWM output 
  setMotorSpeed(0);     //stop motor

  while(Serial.read() >= 0){};
}

void loop() 
{
  if(isScanning){
    if (lidar.waitScanDot() == RESULT_OK) {
        float distance = lidar.getCurrentScanPoint().distance; //distance value in mm unit
        float angle    = lidar.getCurrentScanPoint().angle; //anglue value in degree
        byte  quality  = lidar.getCurrentScanPoint().quality; //quality of the current measurement
        //bool  startBit = lidar.getCurrentScanPoint().startBit;

        if ((distance >= YDLIDAR_MINDIST) &&  (distance <= YDLIDAR_MAXDIST))  
        {
          //Serial.print("current angle:");
          Serial.print(angle);
          //Serial.print("current distance:");
          Serial.print("       ");
          Serial.print(distance);
          Serial.print(" mm       ");
          Serial.println(quality);
        }
    }else{
        //Serial.println(" YDLIDAR get Scandata failed!!");
    }
  }else
  {
    setMotorSpeed(0);     //stop motor
    restartScan();
  }
}

void setMotorSpeed(float vol)
{
  analogWrite(YDLIDAR_MOTOR_SCTP, (uint8_t)(51*vol));
}


void restartScan()
{
  device_info deviceinfo;
  if (lidar.getDeviceInfo(deviceinfo, 100) == RESULT_OK) 
  {
    Serial.println();
    Serial.println();
    Serial.print("Modell: ");
    Serial.println(deviceinfo.model);

    uint16_t maxv = (uint16_t)(deviceinfo.firmware_version>>8);
    uint16_t midv = (uint16_t)(deviceinfo.firmware_version&0xff)/10;
    uint16_t minv = (uint16_t)(deviceinfo.firmware_version&0xff)%10;
    if(midv==0){
      midv = minv;
      minv = 0;
    }

    Serial.print("Firmware version: ");
    Serial.print(maxv,DEC);
    Serial.print(".");
    Serial.print(midv,DEC);
    Serial.print(".");
    Serial.println(minv,DEC);

    Serial.print("Hardware version: ");
    Serial.println((uint16_t)deviceinfo.hardware_version,DEC);

    Serial.print("Serial: ");
    for (int i=0;i<16;i++){
      Serial.print(deviceinfo.serialnum[i]&0xff, DEC);
    }

    Serial.println();
    Serial.println();
    delay(100);

    isScanning = true;
    //start motor in 1.8v
    setMotorSpeed(YDLIDAR_M_CTR_VOLT);
    Serial.println("Now YDLIDAR is scanning ...\r\n");
  }else
  {
    Serial.println("YDLIDAR get DeviceInfo Error!\r\n");
    isScanning = false;
  }
}

