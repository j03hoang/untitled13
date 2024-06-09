#ifndef UNTITLED13_IMU_H
#define UNTITLED13_IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "config.h"

class IMU;
extern IMU gyro;

Adafruit_BNO055 imuSensor;

class IMU {
    public:
     void init() {
         imuSensor = Adafruit_BNO055(BNO_SENSOR_ID, BNO_ADDRESS, &Wire);

         imuSensor.begin();
         imuSensor.setExtCrystalUse(true);

         m_prev_time = millis();

         reset();
     }

     void reset() {
         m_robot_angle = 0;
     }

     void update() {
         sensors_event_t event;
         imuSensor.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);

         unsigned long currentTime = millis();
         float deltaTime = (float) (currentTime - m_prev_time) / 1000.f;
         m_prev_time = currentTime;

         float z = event.gyro.z;
         Serial.print("z");
         Serial.println(z);

         Serial.println(event.gyro.x);
         Serial.println(event.gyro.y);
         m_rot_change = event.gyro.z * deltaTime;
         m_robot_angle += m_rot_change;

         Serial.print("Rotational Change: ");
         Serial.println(m_rot_change);
     }

     float getRotChange() const {
         return m_rot_change;
     }

    private:
     float m_robot_angle;
     float m_rot_change;
     unsigned long m_prev_time;
};


#endif
