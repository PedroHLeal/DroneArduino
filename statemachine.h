#ifndef STATE_MACHINE
#define STATE_MACHINE

#include "motors.h"
#include "proximitysensor.h"
#include "gyro.h"

#define GETTING_BACK_TO_PLACE 1
#define STATE_LED 13

#define POS_DIFF_INTENSITY 2
#define VEL_DIFF_INTENSITY 18
#define BIAS_CORRECTION_INTENSITY 0.06
#define HOVER_BASIS_INTENSITY 40


typedef struct {
  bool is_on = false;

  float frontIntensity;
  float rearIntensity;
  float leftIntensity;
  float rightIntensity;

  float FLIntensity;
  float FRIntensity;
  float RLIntensity;
  float RRIntensity;

  float targetAngVelPitch, targetAngVelRoll;
  float targetAngAccPitch, targetAngAccRoll;
  float biasCorrectionPitch = 0, biasCorrectionRoll = 0;
} DronePosition;

// -------------------------------------------
// -------------------------------------------

class State {
  public:
    ProximitySensor* ps1;
    Gyro *g;
    Motors* m;
    int next_state = 0;
    virtual void run(DronePosition *d, float dt) = 0;
    State();
  protected:
    void calculatePid(DronePosition *p, float dt);
    void writeMotors(DronePosition *p);
};

class OffState : public State {
  public:
    OffState() : State() {}
    void run(DronePosition *d, float dt);
  private:
    float turningOnTime = 0;
};

class OnState : public State {
  public:
    OnState() : State() {}
    void run(DronePosition *d, float dt);
  private:
    float turningOffTime = 0;
};

// STATE MACHINE ---------------------------------
// -----------------------------------------------

class StateMachine {
  private:
    ProximitySensor *ps1;
  public:
    State *current_state;
    State* states[2];
    DronePosition *d;
    StateMachine(DronePosition *d);
    void run(float dt);
};

#endif