#include <Servo.h>

#ifndef MOTORS
#define MOTORS

// SETTING MOTOR PINS
#define MOTOR_FL 11
#define MOTOR_FR 5
#define MOTOR_RL 6
#define MOTOR_RR 3

#define MIN_PW 1000
#define MAX_PW 2000

class Motors {
  private:
    Servo motor_fl, motor_fr, motor_rl, motor_rr;
  public:
    Motors();
    void writeLeft(int intensity);
    void writeRight(int intensity);
    void writeFront(int intensity);
    void writeRear(int intensity);
    void writeFL(int intensity);
    void writeFR(int intensity);
    void writeRL(int intensity);
    void writeRR(int intensity);
    void writeAll(int intensity);
};

class MotorsSingleton {
  public:
    static Motors* m;
    static Motors* getInstance() {
      if (m == NULL) {
        m = new Motors();
      }

      return m;
    }
};

Motors* MotorsSingleton::m = NULL;
#endif