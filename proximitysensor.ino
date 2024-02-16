void ProximitySensor::setup() {
  front_sensor = new NewPing(FRONT_SONAR_TRIG, FRONT_SONAR_ECHO, FRONT_SONAR_MAX_DISTANCE);
}

float* ProximitySensor::getDistance() {
  distance[0] = front_sensor->ping_cm();
  return distance;
}
