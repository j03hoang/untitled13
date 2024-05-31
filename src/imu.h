#ifndef UNTITLED13_IMU_H
#define UNTITLED13_IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "config.h"

class IMU;

extern IMU imu_;

Adafruit_BNO055 bno;
sensors_event_t event;

class IMU {
    public:
     void init() {
         bno = Adafruit_BNO055(BNO_SENSOR_ID, BNO_ADDRESS, &Wire);
         bno.begin();
         bno.setExtCrystalUse(true);
     }

     void get() {
         bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
         int x = event.gyro.x;
         int y = event.gyro.y;
         Serial.print(x);
         Serial.print(y);

     }
};


#endif
