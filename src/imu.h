#ifndef UNTITLED13_IMU_H
#define UNTITLED13_IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "config.h"

/**
 * https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration
 * https://forums.adafruit.com/viewtopic.php?t=192881
 * 
 */

Adafruit_BNO055 imuSensor(BNO_SENSOR_ID, BNO_ADDRESS, &Wire);

class IMU {
    public:
     void init() {
         imuSensor.begin();
         imuSensor.setExtCrystalUse(true);

         reset();
     }

     void reset() {
         m_prev_angle = 0;
         m_current_angle = 0;
     }

     void update() {
         sensors_event_t event;
         imuSensor.getEvent(&event);
         m_prev_angle = m_current_angle;
         m_current_angle = event.orientation.x;
     }

     float getPrevAngle() const {
         return m_prev_angle;
     }

     float getRotChange() const {
         return m_current_angle;
     }

    private:
     float m_prev_angle;
     float m_current_angle;
};

extern IMU gyro;


#endif
