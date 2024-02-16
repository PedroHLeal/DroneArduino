#include <Wire.h>
#include "proximitysensor.h"
#include "gyro.h"
#include "motors.h"
#include "statemachine.h"

#define TEST_MOTORS 0

Motors* motors;
Gyro* g;
ProximitySensor* ps1;
StateMachine* sm;
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
    // motors->writeAll(180);
    // delay(3);
    // motors->writeAll(0);
    // delay(2);
    // motors->writeAll(20);
    // delay(2);
    // motors->writeAll(0);
    // delay(9999);
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

  motors->writeAll(0);
  delay(2000);
}

void loop() {
  if (TEST_MOTORS) {
    // testMotorsPrint();
  } else {
    currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;

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

    lastTime = currentTime;
  }

}