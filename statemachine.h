#ifndef STATE_MACHINE
#define STATE_MACHINE

#include <SoftwareSerial.h>
#include "motors.h"
#include "proximitysensor.h"
#include "gyro.h"
#include "Bluetooth.h"
#include "utils.h"

#define GETTING_BACK_TO_PLACE 1
#define STATE_LED 13

#define P_ANGLE_GAIN 10
#define P_ANG_VEL_GAIN 0.08
#define I_ANG_VEL_GAIN 0.0015
#define D_ANG_VEL_GAIN 0.8

#define YAW_POS_DIFF_INTENSITY 1
#define MAX_TURN 20

typedef struct {
  bool is_on = false;
  float throttle = 0;

  float frontIntensity;
  float rearIntensity;
  float leftIntensity;
  float rightIntensity;

  float FLIntensity;
  float FRIntensity;
  float RLIntensity;
  float RRIntensity;
  
  float targetAngVelPitch, targetAngVelRoll;
  float 
    angVelPitchError,
    angVelRollError;
    
  float angVelPrevPitchError = 0, angVelPrevRollError = 0;

  float targetAngAccPitch, targetAngAccRoll;
  float targetAngVelYaw = 0;
  float 
    biasCorrectionPitch = 0,
    biasCorrectionRoll = 0;

  float setPointPitch = 0, setPointRoll = 0;
} DronePosition;

// -------------------------------------------
// -------------------------------------------

class State {
  public:
    Gyro *g;
    Motors* m;
    AltSoftSerial* b;
    char btReading[14];
    int btCurrent = 0;
    int next_state = 0;
    virtual void run(DronePosition *d, float dt) = 0;
    void resetPid(DronePosition* d);
    State();
  protected:
    void calculatePid(DronePosition *p, float dt);
    void writeMotors(DronePosition *p);
    void getRemoteCommands(DronePosition *d, float dt);
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
    float 
      turningOffTime = 0,
      emergencyQuitTime = 0,
      takingOffTime = 0;
};

// STATE MACHINE ---------------------------------
// -----------------------------------------------

class StateMachine {
  public:
    State *current_state;
    State* states[2];
    DronePosition *d;
    StateMachine(DronePosition *d);
    void run(float dt);
};

#endif