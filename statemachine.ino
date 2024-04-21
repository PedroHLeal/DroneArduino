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
  b = BluetoothSingleton::getBluetooth();
}

void State::resetPid(DronePosition *d) {
  d->throttle = 0;
  d->biasCorrectionPitch = 0;
  d->biasCorrectionRoll = 0;
  d->setPointRoll = 0;
  d->setPointPitch = 0;
  d->angVelRollError = 0;
  d->angVelPitchError = 0;
  d->angVelPrevPitchError = 0;
  d->angVelPrevRollError = 0;
  d->frontIntensity = 0;
  d->rearIntensity = 0;
  d->leftIntensity = 0;
  d->rightIntensity = 0;
  d->targetAngVelYaw = 0;
  g->gPosZ = 0;
}

void State::getRemoteCommands(DronePosition *d, float dt) {
  if (b->available()) {
    btReading[btCurrent] = b->read();
    btCurrent ++;
    if (btReading[btCurrent - 1] == '\n') {
      int c = 0;
      char value[4];

      // reading throttle
      int v = 0;
      while (c < btCurrent) {
        if (btReading[c] != ' ') {
          value[v] = btReading[c];
        } else {
          value[v] = '\n';
          break;
        }
        v++;
        c++;
      }

      d->throttle = String(value).toInt();

      // reading pitch
      v = 0;
      c++;
      while (c < btCurrent) {
        if (btReading[c] != ' ') {
          value[v] = btReading[c];
        } else {
          value[v] = '\n';
          break;
        }
        v++;
        c++;
      }
      d->setPointPitch = (- String(value).toInt());

      // reading roll
      v = 0;
      c++;
      while (c < btCurrent) {
        if (btReading[c] != ' ') {
          value[v] = btReading[c];
        } else {
          value[v] = '\n';
          break;
        }
        v++;
        c++;
      }
      d->setPointRoll = (String(value).toInt());

      Serial.println(String(d->throttle) + " " + String(d->setPointPitch) + " " + String(d->setPointRoll));
      // Serial.println(String(d->FLIntensity) + " " + String(d->FRIntensity) + " " + String(d->RLIntensity) + " " + String(d->RRIntensity));
      btCurrent = 0;
    }
  }
}

void State::calculatePid(DronePosition *d, float dt) {

  d->targetAngVelPitch = (d->setPointPitch - g->gPosX) * P_ANGLE_GAIN;
  d->targetAngVelRoll = (d->setPointRoll - g->gPosY) * P_ANGLE_GAIN;
  d->targetAngVelYaw = g->gPosZ * YAW_POS_DIFF_INTENSITY;

  // Serial.println(String(d->targetAngVelPitch) + " " + String(d->targetAngVelRoll));

  d->angVelPitchError = d->targetAngVelPitch - g->gX;
  d->angVelRollError = d->targetAngVelRoll - g->gY;

  d->biasCorrectionPitch 
    += (d->angVelPitchError + d->angVelPrevPitchError) * I_ANG_VEL_GAIN / 2;
  d->biasCorrectionRoll 
    += (d->angVelRollError + d->angVelPrevRollError) * I_ANG_VEL_GAIN / 2;

  d->biasCorrectionPitch = constrain(d->biasCorrectionPitch, -5, 5);
  d->biasCorrectionRoll = constrain(d->biasCorrectionRoll, -5, 5);

  // Serial.println(String(d->biasCorrectionPitch) + " " + String(d->biasCorrectionRoll));

  d->targetAngAccPitch
    = constrain(
      (d->targetAngVelPitch - g->gX) * P_ANG_VEL_GAIN +
      d->biasCorrectionPitch +
      (d->angVelPitchError - d->angVelPrevPitchError) * D_ANG_VEL_GAIN,
     -MAX_TURN/2, MAX_TURN/2);

  d->targetAngAccRoll
    = constrain(
      (d->targetAngVelRoll - g->gY) * P_ANG_VEL_GAIN +
      d->biasCorrectionRoll +
      (d->angVelRollError - d->angVelPrevRollError) * D_ANG_VEL_GAIN,
     -MAX_TURN/2, MAX_TURN/2);
    
  d->angVelPrevPitchError = d->angVelPitchError;
  d->angVelPrevRollError = d->angVelRollError;

  d->frontIntensity = d->targetAngAccPitch;
  d->rearIntensity = - d->targetAngAccPitch;
  d->rightIntensity = - d->targetAngAccRoll;
  d->leftIntensity = d->targetAngAccRoll;
}

void State::writeMotors(DronePosition *d) {
  // float nt = 
  //   d->throttle /
  //   cos(sqrt(g->gPosX * g->gPosX + g->gPosY * g->gPosY) * DEG_TO_RAD);
  // Serial.println(g->gPosX + g->gPosY);
  float nt = d->throttle;
  // Serial.println(nt);
  d->FLIntensity = nt + d->leftIntensity + d->frontIntensity + d->targetAngVelYaw;
  d->FRIntensity = nt + d->rightIntensity + d->frontIntensity - d->targetAngVelYaw;
  d->RLIntensity = nt + d->leftIntensity + d->rearIntensity - d->targetAngVelYaw;
  d->RRIntensity = nt + d->rightIntensity + d->rearIntensity + d->targetAngVelYaw;

  m->writeFL(d->FLIntensity);
  m->writeFR(d->FRIntensity);
  m->writeRL(d->RLIntensity);
  m->writeRR(d->RRIntensity);

  // Serial.println(String(d->FLIntensity) + " " + String(d->FRIntensity) + " " + String(d->RLIntensity) + " " + String(d->RRIntensity));
}

// OFFSTATE --------------------------------------------------------
// -----------------------------------------------------------------

void OffState::run(DronePosition *d, float dt) {
  // d->throttle = 65;
  getRemoteCommands(d, dt);
  calculatePid(d, dt);

  if (d->throttle == 0) {
    resetPid(d);
  }
  writeMotors(d);
}

// ONSTATE --------------------------------------------------------
// -----------------------------------------------------------------

void OnState::run(DronePosition *d, float dt) {
  getRemoteCommands(d, dt);
  calculatePid(d, dt);
  writeMotors(d);
}
