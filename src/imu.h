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

         m_prev_time = 0;
         m_robot_angle = 0;

         reset();
     }

     void reset() {
         m_robot_angle = 0;
     }

     void update() {
         unsigned long currentTime = millis();
         float deltaTime = (float) (currentTime - m_prev_time) / 1000.f;
         m_prev_time = currentTime;


         // calibration
         uint8_t system, gyro, accel, mag;
         system = gyro = accel = mag = 0;
         imuSensor.getCalibration(&system, &gyro, &accel, &mag);
         Serial.print("\t");
         if (!system)
         {
             Serial.print("! ");
         }

         /* Display the individual values */
         Serial.print("Sys:");
         Serial.print(system, DEC);
         Serial.print(" G:");
         Serial.print(gyro, DEC);
         Serial.print(" A:");
         Serial.print(accel, DEC);
         Serial.print(" M:");
         Serial.print(mag, DEC);



         // TEST1
         imu::Vector<3> gyr = imuSensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

         double z = gyr.z();
         Serial.print("z: ");
         Serial.println(z);

         Serial.print("x: ");
         Serial.println(gyr.x());

         Serial.print("y: ");
         Serial.println(gyr.y());

         double test = z * deltaTime;
         Serial.print("rotational change: ");
         Serial.println(test);
         m_robot_angle += test;
         Serial.print("Angle: ");
         Serial.println(m_robot_angle );

         // TEST2

//         sensors_event_t event;
//         imuSensor.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);

//         m_rot_change = event.gyro.z * deltaTime;
//         m_robot_angle += m_rot_change;
//
//
//         Serial.print("Angle: ");
//         Serial.println(m_robot_angle);
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
