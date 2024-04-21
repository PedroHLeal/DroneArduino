#include <Wire.h>
#include "proximitysensor.h"
#include "gyro.h"
#include "motors.h"
#include "statemachine.h"
#include "utils.h"
#include "storage.h"

#define RUN_DRONE 0
#define CALIBRATE 3
#define SHOW_CALIBRATION_VALUES 4

#define RUNNING_PROGRAM 0
Motors* motors;
Gyro* g;
StateMachine* sm;
bool calibrating = true;
int read = 0;
float currentTime, lastTime = 0;
bool printCalibrate = true;
DronePosition d;
uint32_t loopTimer;

void testMotorsPrint() {
  if (Serial.available() > 0) {
    int value = Serial.parseInt();
    motors->writeAll(value);
  }
}

void runDrone(float dt) {
  g->updateData(dt);
  sm->run(dt);
  while (micros() - loopTimer < 4000);
  loopTimer=micros();
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

  digitalWrite(9, HIGH);

  motors = MotorsSingleton::getInstance();
  g = GyroSingleton::getGyro();
  if (RUNNING_PROGRAM == RUN_DRONE) {
    sm = new StateMachine(&d);

    motors->writeAll(180);
    delay(5000);
    
    motors->writeAll(0);
    delay(10000);

    // motors->writeFR(20);
    // delay(2000);

    // motors->writeFL(20);
    // delay(2000);

    // motors->writeRR(20);
    // delay(2000);

    // motors->writeRL(20);
    // delay(2000);

    // motors->writeAll(0);
    // delay(999999);
  }
  loopTimer = micros();
}

void loop() {
  currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  
  switch (RUNNING_PROGRAM) {
    case RUN_DRONE:
      runDrone(dt);
      break;
    case CALIBRATE:
      calibrate();
      break;
    case SHOW_CALIBRATION_VALUES:
      showCalibrationValues();
      delay(999999999);
      break;
  }
  lastTime = currentTime;

}