// void ProximitySensor::setup() {
//   front_sensor = new NewPing(FRONT_SONAR_TRIG, FRONT_SONAR_ECHO, FRONT_SONAR_MAX_DISTANCE);
// }

// float* ProximitySensor::getDistance() {
//   rawDistance[0] = front_sensor->ping_cm();
//   return rawDistance;
// }

// float* ProximitySensor::getLowPassFilteredDistance(float dt) {
//     float distance0 = front_sensor->ping_cm();
//     filteredDistance[0]
//       = low_pass(2, dt, distance0, distance1, distance2, distanceFinal1, distanceFinal2);

//     distance2 = distance1;
//     distance1 = distance0;
//     distanceFinal2 = distanceFinal1;
//     distanceFinal1 = filteredDistance[0];

//     return filteredDistance;
// }