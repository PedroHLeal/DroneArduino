#ifndef PROXIMITY_SENSOR
#define PROXIMITY_SENSOR

#define FRONT_SONAR_ECHO 10
#define FRONT_SONAR_TRIG 9
#define FRONT_SONAR_MAX_DISTANCE 60

#include <NewPing.h>

class ProximitySensor {
  public:
    NewPing *front_sensor;
    void setup();
    float* getDistance();
    float* getLowPassFilteredDistance(float dt);
  private:
    float rawDistance[1];
    float filteredDistance[1];
    float distance0, distance1, distance2, distanceFinal1, distanceFinal2;
};

class ProximitySensorSingleton {
  public:
    static ProximitySensor *p;
    static ProximitySensor* getProximitySensor() {
      if (p == NULL) {
        p = new ProximitySensor();
        p->setup();
      }
      return p;
    }
};

ProximitySensor* ProximitySensorSingleton::p = NULL;
#endif