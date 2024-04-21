#include <Wire.h>
#include "utils.h"
#include "storage.h"

#define LOW_PASS_SENSITIVITY 2
const int MPU=0x68; 

Gyro::Gyro(bool shouldCalibrate = false) {
  Wire.begin();
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);    
  Wire.endTransmission(true);
  this->shouldCalibrate = shouldCalibrate;
}

void Gyro::readRawValues() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);
  
  rawAX = double(Wire.read()<<8|Wire.read()) / 16384.0; // accX
  rawAY = double(Wire.read()<<8|Wire.read()) / 16384.0; // accY
  rawAZ = double(Wire.read()<<8|Wire.read()) / 16384.0; // accZ
  tmp = double(Wire.read()<<8|Wire.read()); // tmp
  rawGX = double(Wire.read()<<8|Wire.read()) / 131.0; // gyroX
  rawGY = double(Wire.read()<<8|Wire.read()) / 131.0; // gyroY
  rawGZ = double(Wire.read()<<8|Wire.read()) / 131.0; // gyroZ
}

bool Gyro::updateData(float dt) {
  this->readRawValues();

  aX = rawAX - this->aErrorX;
  aY = rawAY - this->aErrorY;
  aZ = rawAZ;
  gX = rawGX - this->gErrorX;
  gY = rawGY - this->gErrorY;
  gZ = rawGZ - this->gErrorZ;

  this->setAngle(dt);
  
  return true;
}

bool Gyro::calibrate() {
  this->readRawValues();
  if (this->currentRound < this->calibrationRounds) {
    aErrorX += rawAX;
    aErrorY += rawAY;
    aErrorZ += rawAZ;
    gErrorX += rawGX;
    gErrorY += rawGY;
    gErrorZ += rawGZ;

    this->currentRound ++;
    return false;
  } else {
    gErrorX /= calibrationRounds;
    gErrorY /= calibrationRounds;
    gErrorZ /= calibrationRounds;

    aErrorX /= calibrationRounds;
    aErrorY /= calibrationRounds;
    aErrorZ /= calibrationRounds;

    saveCalibration(gErrorX, gErrorY, gErrorZ, aErrorX, aErrorY);
    return true;
  }
}

void Gyro::readCalibration() {
  loadCalibration(&gErrorX, &gErrorY, &gErrorZ, &aErrorX, &aErrorY);
}

void Gyro::setAngle(float dt) {
  // Detect accel angle
  float aAngleX 
    = (atan(aY / sqrt(sq(aX) + sq(aZ)))) * RAD_TO_DEG;
  float aAngleY 
    = (atan(-1 * aX / sqrt(sq(aY) + sq(aZ)))) * RAD_TO_DEG;
  float aAngleZ = (atan2(aY, aZ)) * RAD_TO_DEG;

  gPosX = 0.98 * (gPosX + gX * dt) + 0.02 * aAngleX;
  gPosY = 0.98 * (gPosY + gY * dt) + 0.02 * aAngleY;
  gPosZ += gZ * dt;

  // Serial.println(String(aX) + " " + String(aY));
  // Serial.println(String(aAngleX) + " " + String(aAngleY));
  // Serial.println(String(gPosX) + " " + String(gPosY));
}
