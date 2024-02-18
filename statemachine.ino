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

  d->biasCorrectionPitch = constrain(d->biasCorrectionPitch, -120, 120);
  d->biasCorrectionRoll = constrain(d->biasCorrectionRoll, -120, 120);

  d->targetAngAccPitch
    = d->targetAngVelPitch - g->angularVelX * VEL_DIFF_INTENSITY + d->biasCorrectionPitch;
  d->targetAngAccRoll
    = d->targetAngVelRoll - g->angularVelY * VEL_DIFF_INTENSITY + d->biasCorrectionRoll;

  d->frontIntensity = - d->targetAngAccPitch;
  d->rearIntensity = d->targetAngAccPitch;
  d->rightIntensity = - d->targetAngAccRoll;
  d->leftIntensity = d->targetAngAccRoll;
}

void State::writeMotors(DronePosition *d) {
  d->FLIntensity = HOVER_BASIS_INTENSITY + d->leftIntensity + d->frontIntensity;
  d->FRIntensity 
    = HOVER_BASIS_INTENSITY + d->rightIntensity + d->frontIntensity;
  d->RLIntensity = HOVER_BASIS_INTENSITY + d->leftIntensity + d->rearIntensity;
  d->RRIntensity = HOVER_BASIS_INTENSITY + d->rightIntensity + d->rearIntensity;

  m->writeFL(constrain(d->FLIntensity, 20, 145));
  m->writeFR(constrain(d->FRIntensity, 20, 145));
  m->writeRL(constrain(d->RLIntensity, 20, 145));
  m->writeRR(constrain(d->RRIntensity, 20, 145));

  Serial.println(String(d->FLIntensity) + " " + String(d->FRIntensity) + " " + String(d->RLIntensity) + " " + String(d->RRIntensity));
}

// OFFSTATE --------------------------------------------------------
// -----------------------------------------------------------------

void OffState::run(DronePosition *d, float dt) {
  float distance = ps1->getLowPassFilteredDistance(dt)[0];
  Serial.println(distance);
  if (distance > 0 && distance < 20) {
    turningOnTime += dt;
  } else {
    turningOnTime = 0;
  }

  if (turningOnTime > 3) {
    next_state = 1;
    turningOnTime = 0;
  }
}

// ONSTATE --------------------------------------------------------
// -----------------------------------------------------------------

void OnState::run(DronePosition *d, float dt) {
  float distance = ps1->getLowPassFilteredDistance(dt)[0];
  if (distance > 0 && distance < 20) {
    turningOffTime += dt;
  } else {
    turningOffTime = 0;
  }

  calculatePid(d, dt);
  writeMotors(d);

  if (turningOffTime > 3) {
    next_state = 0;
    turningOffTime = 0;
  }
}
