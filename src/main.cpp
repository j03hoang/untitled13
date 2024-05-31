#include <Arduino.h>
#include <RotaryEncoder.h>

//#include <QuickPID.h>
//
//float Setpoint, Input, Output;
//
//float Kp = 2, Ki = 5, Kd = 1;
//QuickPID myPID(&Input, &Output, &Setpoint);


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (1000)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


#define PIN_IN1 A2
#define PIN_IN2 A3

const int MOTOR_A_FWD = 7;
const int MOTOR_A_REVERSE = 8;
const int MOTOR_B_FWD = 9;
const int MOTOR_B_REVERSE = 10;

const int ULTRASONIC_TRIG_1 = 4;
const int ULTRASONIC_ECHO_1 = PIN_A4;

const int ROT_PER_TICK = 617;
const int DIST_PER_ROT = -1;

const int MAX_PWR = 200;
const int LOW_ = 0;


RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);


// the maximum acceleration is 10 times.
constexpr float m = 10;

// at 200ms or slower, there should be no acceleration. (factor 1)
constexpr float longCutoff = 50;

// at 5 ms, we want to have maximum acceleration (factor m)
constexpr float shortCutoff = 5;

// To derive the calc. constants, compute as follows:
// On an x(ms) - y(factor) plane resolve a linear formular factor(ms) = a * ms + b;
// where  f(4)=10 and f(200)=1

constexpr float a = (m - 1) / (shortCutoff - longCutoff);
constexpr float b = 1 - longCutoff * a;

// a global variables to hold the last position
static int lastPos = 0;

//void setReports(void) {
//    Serial.println("Setting desired reports");
//    if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
//        Serial.println("Could not enable game vector");
//    }
//}

void doGyro() {
    sensors_event_t event;
    bno.getEvent(&event);

    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);


    /* New line for the next sample */
    Serial.println("");

    /* Wait the specified delay before requesting nex data */
    delay(BNO055_SAMPLERATE_DELAY_MS);
}

void daMotor() {
    Serial.println("FORWARD");
    analogWrite(MOTOR_B_FWD, MAX_PWR / 2);
    analogWrite(MOTOR_B_REVERSE, LOW_);

    analogWrite(MOTOR_A_FWD, MAX_PWR / 2);
    analogWrite(MOTOR_A_REVERSE, LOW_);


    delay(3000);

    Serial.println("Stop");
    analogWrite(MOTOR_B_FWD, MAX_PWR);
    analogWrite(MOTOR_B_REVERSE, MAX_PWR);

    analogWrite(MOTOR_A_FWD, MAX_PWR);
    analogWrite(MOTOR_A_REVERSE, MAX_PWR);
    delay(3000);


    Serial.println("REVERSE");
    analogWrite(MOTOR_B_FWD, LOW_);
    analogWrite(MOTOR_B_REVERSE, MAX_PWR);

    analogWrite(MOTOR_A_FWD, LOW_);
    analogWrite(MOTOR_A_REVERSE, MAX_PWR);
    delay(3000);


    Serial.println("Stop");
    analogWrite(MOTOR_B_FWD, MAX_PWR);
    analogWrite(MOTOR_B_REVERSE, MAX_PWR);

    analogWrite(MOTOR_A_FWD, MAX_PWR);
    analogWrite(MOTOR_A_REVERSE, MAX_PWR);
    delay(3000);

    Serial.println("TURN AROUND");
    analogWrite(MOTOR_B_FWD, MAX_PWR);
    analogWrite(MOTOR_B_REVERSE, LOW_);

    analogWrite(MOTOR_A_FWD, LOW_);
    analogWrite(MOTOR_A_REVERSE, MAX_PWR);
    delay(1000);

    Serial.println("TURN RIGHT");
    analogWrite(MOTOR_B_FWD, MAX_PWR);
    analogWrite(MOTOR_B_REVERSE, LOW_);

    analogWrite(MOTOR_A_FWD, LOW_);
    analogWrite(MOTOR_A_REVERSE, LOW_);
    delay(1000);

    Serial.println("TURN LEFT");
    analogWrite(MOTOR_B_FWD, LOW_);
    analogWrite(MOTOR_B_REVERSE, LOW_);

    analogWrite(MOTOR_A_FWD, MAX_PWR);
    analogWrite(MOTOR_A_REVERSE, LOW_);
    delay(1000);
}

double getDistance() {
    unsigned long duration, distance;
    digitalWrite(ULTRASONIC_TRIG_1, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_1, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_ECHO_1, LOW);
    duration = pulseIn(ULTRASONIC_ECHO_1, HIGH);
    distance = (duration/2) / 29.1;

    delay(100);

    return distance;
}

double convert(int numTicks) {
    double numRotations = (double) numTicks / ROT_PER_TICK;
    return numRotations * 135;
}

void encoderDo() {
    encoder.tick();

    int newPos = encoder.getPosition();
    if (lastPos != newPos) {

        // accelerate when there was a previous rotation in the same direction.

        unsigned long ms = encoder.getMillisBetweenRotations();

        if (ms < longCutoff) {
            // do some acceleration using factors a and b

            // limit to maximum acceleration
            if (ms < shortCutoff) {
                ms = shortCutoff;
            }

            float ticksActual_float = a * ms + b;
            Serial.print("  f= ");
            Serial.println(ticksActual_float);

            long deltaTicks = (long)ticksActual_float * (newPos - lastPos);
            Serial.print("  d= ");
            Serial.println(deltaTicks);

            newPos = newPos + deltaTicks;
            encoder.setPosition(newPos);
        }

        Serial.print(newPos);
        Serial.print("  ms: ");
        Serial.println(ms);
        lastPos = newPos;

        Serial.print(convert(newPos));
        Serial.println(" distance traveled");
    }
}

void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
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
}

void init_imu() {
    Serial.begin(9600);

    while (!Serial) delay(10);  // wait for serial port to open!

    Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    delay(1000);

    /* Display some basic information on this sensor */
    displaySensorDetails();

    bno.setExtCrystalUse(true);
}

void init_motor() {
    pinMode(MOTOR_A_FWD, OUTPUT);
    pinMode(MOTOR_A_REVERSE, OUTPUT);
    pinMode(MOTOR_B_FWD, OUTPUT);
    pinMode(MOTOR_B_REVERSE, OUTPUT);
}

void init_sensor() {
    pinMode(ULTRASONIC_TRIG_1, OUTPUT);
    pinMode(ULTRASONIC_ECHO_1, INPUT);
}

void setup() {
    init_imu();
}


void loop() {
    doGyro();
}