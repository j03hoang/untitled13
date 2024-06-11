#ifndef UNTITLED13_IMU_H
#define UNTITLED13_IMU_H

#include <Wire.h>
#include "../lib/FastIMU/src/F_MPU6500.hpp"
#include "config.h"
/**
 * Library: https://github.com/LiquidCGS/FastIMU
 *
 * https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration
 * https://forums.adafruit.com/viewtopic.php?t=192881
 * 
 */

GyroData gyroData;
MPU6500 imuSensor;
calData calib = { 0 };

class IMU {
    public:
     void init() {
         Wire.begin();
         Wire.setClock(400000);
         imuSensor.init(calib, IMU_ADDRESS);

         reset();
     }

     void reset() {
         m_prev_angle = 0;
         m_current_angle = 0;
         m_prev_time = 0;
     }

     void update() {
         imuSensor.update();
         imuSensor.getGyro(&gyroData);

         unsigned long currentTime = millis();
         float deltaTime =  ((float) currentTime - m_prev_time) / 1000.f;
         m_prev_time = (float) currentTime;

         m_rot_change = gyroData.gyroZ * deltaTime;

         m_current_angle += m_rot_change;

         m_current_angle = fmod(m_current_angle, 360.0f);

         m_prev_angle = m_current_angle;
     }


    /** a previous angle is maintained to keep track of how much rotation change is needed for a turn*/
     float getPrevAngle() const {
         return m_prev_angle;
     }

     float getRotChange() const {
         return m_current_angle;
     }

    private:
     float m_prev_time;
     float m_rot_change;
     float m_prev_angle;
     float m_current_angle;
};

extern IMU gyro;


#endif
