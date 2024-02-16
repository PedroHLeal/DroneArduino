#ifndef PROXIMITY_SENSOR
#define PROXIMITY_SENSOR

#define FRONT_SONAR_ECHO 10
#define FRONT_SONAR_TRIG 9
#define FRONT_SONAR_MAX_DISTANCE 40

#include <NewPing.h>

class ProximitySensor {
  public:
    NewPing *front_sensor;
    void setup();
    float* getDistance();
  private:
    float distance[1];
};

class ProximitySensorSingleton {
  public:
    static ProximitySensor *p;
    static ProximitySensor* getProximitySensor() {
      if (p == NULL) {
        p = new ProximitySensor();
      }
      return p;
    }
};

ProximitySensor* ProximitySensorSingleton::p = NULL;
#endif