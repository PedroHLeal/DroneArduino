StateMachine::StateMachine(DronePosition *d) {
  this->d = d;
  states[0] = new HoveringState();
  states[1] = new FindingPositionState();
  this->current_state = states[0];
  this->current_state->next_state = -1;
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
}

void State::run(DronePosition *d, float dt) {
  d->emergencyQuitTime += dt;
  d->targetAngVelPitch = -g->gPosX * POS_DIFF_INTENSITY;
  d->targetAngVelRoll = -g->gPosY * POS_DIFF_INTENSITY;

  d->biasCorrectionPitch += d->targetAngVelPitch * BIAS_CORRECTION_INTENSITY;
  d->biasCorrectionRoll += d->targetAngVelRoll * BIAS_CORRECTION_INTENSITY;

  d->biasCorrectionPitch = constrain(d->biasCorrectionPitch, -120, 120);
  d->biasCorrectionRoll = constrain(d->biasCorrectionRoll, -120, 120);

  // Serial.println(String(d->biasCorrectionPitch) + " " + String(d->biasCorrectionRoll));

  d->targetAngAccPitch
    = d->targetAngVelPitch - g->angularVelX * VEL_DIFF_INTENSITY + d->biasCorrectionPitch;
  d->targetAngAccRoll
    = d->targetAngVelRoll - g->angularVelY * VEL_DIFF_INTENSITY + d->biasCorrectionRoll;

  d->frontIntensity = - d->targetAngAccPitch;
  d->rearIntensity = d->targetAngAccPitch;
  d->rightIntensity = - d->targetAngAccRoll;
  d->leftIntensity = d->targetAngAccRoll;
  
  this->loop_state(d, dt);

  d->FLIntensity = HOVER_BASIS_INTENSITY + d->leftIntensity + d->frontIntensity;
  d->FRIntensity 
    = HOVER_BASIS_INTENSITY + d->rightIntensity + d->frontIntensity;
  d->RLIntensity = HOVER_BASIS_INTENSITY + d->leftIntensity + d->rearIntensity;
  d->RRIntensity = HOVER_BASIS_INTENSITY + d->rightIntensity + d->rearIntensity;

  m->writeFL(d->emergencyQuitTime > 3 ? 0 : constrain(d->FLIntensity, 45, 145));
  m->writeFR(d->emergencyQuitTime > 3 ? 0 : constrain(d->FRIntensity, 45, 145));
  m->writeRL(d->emergencyQuitTime > 3 ? 0 : constrain(d->RLIntensity, 45, 145));
  m->writeRR(d->emergencyQuitTime > 3 ? 0 : constrain(d->RRIntensity, 45, 145));

  Serial.println(String(d->FLIntensity) + " " + String(d->FRIntensity) + " " + String(d->RLIntensity) + " " + String(d->RRIntensity));
}

// ------------------------------------
// HOVERING STATE ---------------------

HoveringState::HoveringState() {
  ps1 = ProximitySensorSingleton::getProximitySensor();
  ps1->setup();
}

void HoveringState::reset() {
  this->led_flicker_counter = 0; 
  this->changing_state_counter = 0;
  digitalWrite(STATE_LED, 0);
}

void HoveringState::loop_state(DronePosition *d, float dt) {
  // float distance = ps1->getDistance()[0];
  // if (distance && distance < 20) {
  //   if (changing_state_counter > 2) {
  //     this->reset();
  //     this->next_state = 1;
  //   }

  //   if (led_flicker_counter > 0.2) {
  //     this->state_led_value = !this->state_led_value;
  //     digitalWrite(STATE_LED, this->state_led_value);
  //     led_flicker_counter = 0;
  //   }

  //   this->led_flicker_counter += dt; 
  //   this->changing_state_counter += dt; 
  // } else {
  //   if (print_time > 0.03) {
  //     print_time = 0;
  //   }
  //   this->reset();
  //   print_time += dt;
  // }
}

// ------------------------------------
// FINDING POSITION STATE -------------

FindingPositionState::FindingPositionState() {
  ps1 = ProximitySensorSingleton::getProximitySensor();
}

void FindingPositionState::reset() {
  counter = 0;
}

void FindingPositionState::loop_state(DronePosition *d, float dt) {
  digitalWrite(STATE_LED, 1);


}
