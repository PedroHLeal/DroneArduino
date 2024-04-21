Motors::Motors() {
  motor_fl.attach(MOTOR_FL);
  motor_fr.attach(MOTOR_FR);
  motor_rl.attach(MOTOR_RL);
  motor_rr.attach(MOTOR_RR);
}

void Motors::writeLeft(int intensity) {
  intensity = map(intensity, 0, 180, 1000, 2000);
  motor_fl.write(intensity);
  motor_rl.write(intensity);
}

void Motors:: writeRight(int intensity) {
  intensity = map(intensity, 0, 180, 1000, 2000);
  motor_fr.write(intensity);
  motor_rr.write(intensity);
}

void Motors::writeFront(int intensity) {
  intensity = map(intensity, 0, 180, 1000, 2000);
  motor_fl.write(intensity);
  motor_fr.write(intensity);
}

void Motors::writeRear(int intensity) {
  intensity = map(intensity, 0, 180, 1000, 2000);
  motor_rl.write(intensity);
  motor_rr.write(intensity);
}

void Motors::writeFL(int intensity) {
  intensity = map(intensity, 0, 180, 1000, 2000);
  motor_fl.write(intensity);
}

void Motors::writeFR(int intensity) {
  intensity = map(intensity, 0, 180, 1000, 2000);
  motor_fr.write(intensity);
}

void Motors::writeRL(int intensity) {
  intensity = map(intensity, 0, 180, 1000, 2000);
  motor_rl.write(intensity);
}

void Motors::writeRR(int intensity) {
  intensity = map(intensity, 0, 180, 1000, 2000);
  motor_rr.write(intensity);
}

void Motors::writeAll(int intensity) {
  intensity = map(intensity, 0, 180, 1000, 2000);
  motor_rl.write(intensity);
  motor_rr.write(intensity);
  motor_fr.write(intensity);
  motor_fl.write(intensity);
}