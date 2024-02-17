#include <Wire.h>
#include "proximitysensor.h"
#include "gyro.h"
#include "motors.h"
#include "statemachine.h"
#include "utils.h"

#define RUN_DRONE 0
#define TEST_MOTORS_SERIAL 1
#define TEST_MOTORS_PROXIMITY_SENSOR 2
#define CALIBRATE 3

#define RUNNING_PROGRAM 2

Motors* motors;
Gyro* g;
StateMachine* sm;
ProximitySensor *ps1;
bool calibrating = true;
int read = 0;
float currentTime, lastTime = 0;
DronePosition d;
bool calibrationLight = false;
bool printCalibrate = false;
float lastRead = 0;

void testMotorsPrint() {
  if (Serial.available() > 0) {
    int value = Serial.parseInt();
    motors->writeAll(value);
  }
}

void testMotors() {
    float distance = ps1->getDistance()[0];
    if (distance > 0 && lastRead > 0) {
      Serial.println(map(distance, 40, 5, 0, 180));
      motors->writeAll(map(distance, 40, 5, 0, 180));
    } else {
      motors->writeAll(0);
    }
    lastRead = distance;
}

void testMotorsProximitySensor(float dt) {
    float distance = ps1->getLowPassFilteredDistance()[0];
    float intensity = constrain(map(distance, 20, 45, 0, 100), 0, 100);
    Serial.println(String(distance) + " " + String(intensity));
    motors->writeAll(intensity);
}

void runDrone(float dt) {
  if (!g->updateData(dt)) {
    if (!printCalibrate) {
      Serial.println("calibrating");
      printCalibrate = true;
    }

    if ((micros() - currentTime)/ 1000000.0 > 2) {
      currentTime = micros();
    }
    // wait until calibrated
    lastTime = currentTime;
    return;
  }

  sm->run(dt);
}

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(50);

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(8, LOW);

  motors = MotorsSingleton::getInstance();
  g = GyroSingleton::getGyro();
  sm = new StateMachine(&d);
  ps1 = ProximitySensorSingleton::getProximitySensor();

  // UNCONMMENT THIS TO RUN MOTOR CALIBRATION PROGRAN
  // motors->writeAll(180);
  // delay(10000);

  
  // motors->writeAll(0);
  // delay(10000);

  
  // motors->writeAll(20);
  // delay(2000);

  // motors->writeAll(0);
  // delay(999999);
  // END IF THE MOTOR CALIBRATION PROGRAM

  motors->writeAll(0);
  delay(2000);
}

void loop() {
  currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  // Serial.println("printing anything for a check");
  // Serial.println(ps1->getDistance()[0]);
  switch (RUNNING_PROGRAM) {
    case RUN_DRONE:
      runDrone(dt);
      break;
    case TEST_MOTORS_SERIAL:
      // testMotorsPrint();
      break;
    case TEST_MOTORS_PROXIMITY_SENSOR:
      testMotorsProximitySensor(dt);
      break;
  }
  lastTime = currentTime;

}