void ProximitySensor::setup() {
  front_sensor = new NewPing(FRONT_SONAR_TRIG, FRONT_SONAR_ECHO, FRONT_SONAR_MAX_DISTANCE);
}

float* ProximitySensor::getDistance() {
  distance[0] = front_sensor->ping_cm();
  return distance;
}

float* ProximitySensor::getDistance() {
  distance[0] = front_sensor->ping_cm();
  return distance;
}

float* ProximitySensor::getLowPassFilteredDistance() {
    float distance0 = front_sensor->ping_cm();
    float filteredDistance[0] 
      = low_pass(2, dt, distance0, distance1, distance2, distanceFinal1, distanceFinal2);

    distance2 = distance1;
    distance1 = distance0;
    distanceFinal2 = distanceFinal1;
    distanceFinal1 = distance;

    return filteredDistance;
}