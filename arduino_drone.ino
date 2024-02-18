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
bool printCalibrate = true;
DronePosition d;

void testMotorsPrint() {
  if (Serial.available() > 0) {
    int value = Serial.parseInt();
    motors->writeAll(value);
  }
}

void testMotorsProximitySensor(float dt) {
    float distance = ps1->getLowPassFilteredDistance(dt)[0];
    float intensity = constrain(map(distance, 20, 45, 0, 100), 0, 100);
    Serial.println(String(distance) + " " + String(intensity));
    motors->writeAll(intensity);
}

void runDrone(float dt) {
  g->updateData(dt);
  sm->run(dt);
}

void calibrate() {
  while (!g->calibrate()) {
    if (printCalibrate) {
      Serial.println("Calibrating");
      printCalibrate = false;
    }
  }
  Serial.println("Calibrated");
  delay(1000000);
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
  ps1 = ProximitySensorSingleton::getProximitySensor();
  if (RUNNING_PROGRAM == RUN_DRONE) {
    sm = new StateMachine(&d);
  }

  // UNCONMMENT THIS TO RUN ESC CALIBRATION PROGRAN
  // motors->writeAll(180);
  // delay(10000);
  
  // motors->writeAll(0);
  // delay(10000);
  
  // motors->writeFL(20);
  // delay(2000);
  // motors->writeFL(0);
  // delay(2000);

  // motors->writeFR(20);
  // delay(2000);
  // motors->writeFR(0);
  // delay(2000);

  // motors->writeRL(20);
  // delay(2000);
  // motors->writeRL(0);
  // delay(2000);

  // motors->writeRR(20);
  // delay(2000);
  // motors->writeRR(0);
  
  // delay(999999);
  // END IF THE MOTOR CALIBRATION PROGRAM

  motors->writeAll(0);
  delay(2000);
}

void loop() {
  currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  
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
    case CALIBRATE:
      calibrate();
      break;
  }
  lastTime = currentTime;

}