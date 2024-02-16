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
#define HOVER_BASIS_INTENSITY 95


typedef struct {
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

  bool emergencyQuit = false;
  float emergencyQuitTime = 0;
} DronePosition;

class State {
  public:
    ProximitySensor* ps1;
    Gyro *g;
    Motors* m;
    int next_state = 0;
    void run(DronePosition *d, float dt);
    virtual void reset() = 0;
    virtual void loop_state(DronePosition *d, float dt) = 0;
    State();
  protected:
    void resetPosition(DronePosition *d);
};

class HoveringState : public State {
  public:
    HoveringState();
    void loop_state(DronePosition *d, float dt) override;
    void reset() override;
  private:
    float print_time = 0;
    int state_led_value = 0;
    float led_flicker_counter = 0;
    float changing_state_counter = 0;
    ProximitySensor *ps1;
};

class FindingPositionState : public State {
  public:
    FindingPositionState();
    float counter = 0;
    void loop_state(DronePosition *d, float dt) override;
    void reset() override;
};

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