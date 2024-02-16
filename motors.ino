Motors::Motors() {
  motor_fl.attach(MOTOR_FL, MIN_PW, MAX_PW);
  motor_fr.attach(MOTOR_FR, MIN_PW, MAX_PW);
  motor_rl.attach(MOTOR_RL, MIN_PW, MAX_PW);
  motor_rr.attach(MOTOR_RR, MIN_PW, MAX_PW);
}

void Motors::writeLeft(int intensity) {
  motor_fl.write(intensity);
  motor_rl.write(intensity);
}

void Motors:: writeRight(int intensity) {
  motor_fr.write(intensity);
  motor_rr.write(intensity);
}

void Motors::writeFront(int intensity) {
  motor_fl.write(intensity);
  motor_fr.write(intensity);
}

void Motors::writeRear(int intensity) {
  motor_rl.write(intensity);
  motor_rr.write(intensity);
}

void Motors::writeFL(int intensity) {
  motor_fl.write(intensity);
}

void Motors::writeFR(int intensity) {
  motor_fr.write(intensity);
}

void Motors::writeRL(int intensity) {
  motor_rl.write(intensity);
}

void Motors::writeRR(int intensity) {
  motor_rr.write(intensity);
}

void Motors::writeAll(int intensity) {
  motor_rl.write(intensity);
  motor_rr.write(intensity);
  motor_fr.write(intensity);
  motor_fl.write(intensity);
}