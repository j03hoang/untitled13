#ifndef UNTITLED13_IMU_H
#define UNTITLED13_IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "config.h"

class IMU;
extern IMU gyro;

Adafruit_BNO055 m_imu = Adafruit_BNO055(55, 0x28, &Wire);

class IMU {
    public:
     void init() {
         m_imu.begin();
         m_imu.setExtCrystalUse(true);

         m_prev_time = millis();
     }

     float getRotChange() {
         sensors_event_t event;
         m_imu.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);

         unsigned long currentTime = millis();
         float deltaTime = (currentTime - m_prev_time) / 1000.;
         m_prev_time = currentTime;

         m_rot_change = event.gyro.z * deltaTime;
         m_robot_angle += m_rot_change;

         Serial.print("Rotational Change: ");
         Serial.println(rotationalChange);

         return rotationalChange;
     }

    private:
     float m_robot_angle;
     float m_rot_change;
     unsigned long m_prev_time;
};


#endif
