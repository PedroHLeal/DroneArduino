StateMachine::StateMachine(DronePosition *d) {
  this->d = d;
  states[0] = new OffState();
  states[1] = new OnState();
  this->current_state = states[0];
  this->current_state->next_state = -1;
  g->readCalibration();
}

void StateMachine::run(float dt) {
  if (current_state->next_state >= 0) {
    this->current_state = states[current_state->next_state];
    current_state->next_state = -1;
  }

  this->current_state->run(d, dt);
}

// ------------------------------------
// BASE STATE -------------------------

State::State() {
  m = MotorsSingleton::getInstance();
  g = GyroSingleton::getGyro();
  ps1 = ProximitySensorSingleton::getProximitySensor();
}

void State::calculatePid(DronePosition *d, float dt) {
  d->targetAngVelPitch = -g->gPosX * POS_DIFF_INTENSITY;
  d->targetAngVelRoll = -g->gPosY * POS_DIFF_INTENSITY;

  d->biasCorrectionPitch += d->targetAngVelPitch * BIAS_CORRECTION_INTENSITY;
  d->biasCorrectionRoll += d->targetAngVelRoll * BIAS_CORRECTION_INTENSITY;

  // d->biasCorrectionPitch = constrain(d->biasCorrectionPitch, -30, 30);
  // d->biasCorrectionRoll = constrain(d->biasCorrectionRoll, -30, 30);

  d->targetAngAccPitch
    = constrain(d->targetAngVelPitch - g->angularVelX * VEL_DIFF_INTENSITY + d->biasCorrectionPitch, -15, 15);
  d->targetAngAccRoll
    = constrain(d->targetAngVelRoll - g->angularVelY * VEL_DIFF_INTENSITY + d->biasCorrectionRoll, -15, 15);

  d->frontIntensity = - d->targetAngAccPitch;
  d->rearIntensity = d->targetAngAccPitch;
  d->rightIntensity = - d->targetAngAccRoll;
  d->leftIntensity = d->targetAngAccRoll;
}

void State::writeMotors(DronePosition *d) {
  d->FLIntensity = d->throttle + d->leftIntensity + d->frontIntensity;
  d->FRIntensity = d->throttle + d->rightIntensity + d->frontIntensity;
  d->RLIntensity = d->throttle + d->leftIntensity + d->rearIntensity;
  d->RRIntensity = d->throttle + d->rightIntensity + d->rearIntensity;

  m->writeFL(d->FLIntensity);
  m->writeFR(d->FRIntensity);
  m->writeRL(d->RLIntensity);
  m->writeRR(d->RRIntensity);

  Serial.println(String(d->FLIntensity) + " " + String(d->FRIntensity) + " " + String(d->RLIntensity) + " " + String(d->RRIntensity));
}

// OFFSTATE --------------------------------------------------------
// -----------------------------------------------------------------

void OffState::run(DronePosition *d, float dt) {
  float distance = ps1->getLowPassFilteredDistance(dt)[0];
  Serial.println(distance);
  if (distance > 1 && distance < 20) {
    turningOnTime += dt;
  } else {
    turningOnTime = 0;
  }

  if (turningOnTime > 3) {
    next_state = 1;
    turningOnTime = 0;
    d->throttle = 70;
  }
}

// ONSTATE --------------------------------------------------------
// -----------------------------------------------------------------

void OnState::run(DronePosition *d, float dt) {
  float distance = ps1->getLowPassFilteredDistance(dt)[0];
  if (distance > 1 && distance < 20) {
    turningOffTime += dt;
  } else {
    turningOffTime = 0;
  }

  calculatePid(d, dt);
  writeMotors(d);
  // emergencyQuitTime += dt;
  takingOffTime += dt;

  if (takingOffTime > 0.5) {
    d->throttle = 70;
  }

  if (turningOffTime > 3 || emergencyQuitTime > 3) {
    next_state = 0;
    turningOffTime = 0;
    motors->writeAll(0);
    d->biasCorrectionPitch = 0;
    d->biasCorrectionRoll = 0;
    d->throttle = 0;
    emergencyQuitTime = 0;
  }
}
