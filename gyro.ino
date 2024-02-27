#include <Wire.h>
#include "utils.h"
#include "storage.h"

#define LOW_PASS_SENSITIVITY 2
const int MPU=0x68; 

Gyro::Gyro(bool shouldCalibrate = false) {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);    
  Wire.endTransmission(true);
  this->shouldCalibrate = shouldCalibrate;
}

void Gyro::readRawValues() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);
  
  rawAX = -double(Wire.read()<<8|Wire.read()) / 16384.0; // accX
  rawAY = -double(Wire.read()<<8|Wire.read()) / 16384.0; // accY
  rawAZ = -double(Wire.read()<<8|Wire.read()) / 16384.0; // accZ
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
    // Serial.println(currentRound);
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

  float nGPosX = 0.98 * (gPosX + gX * dt) + 0.02 * aAngleX;
  float nGPosY = 0.98 * (gPosY + gY * dt) + 0.02 * aAngleY;

  // Serial.println(String(nGPosX) + " " + String(nGPosY));

  angularVelX = nGPosX - gPosX;
  angularVelY = nGPosY - gPosY;

  gPosX = nGPosX; 
  gPosY = nGPosY; 
  gPosZ = 0.98 * (gPosZ + gZ * dt) - 0.02 * aAngleZ;

  gpX += gX * dt; 
  gpY += gY * dt; 
  gpZ += gZ * dt;
}
