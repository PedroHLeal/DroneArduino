#include <Wire.h>
#include "utils.h"

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

bool Gyro::updateData(float dt) {
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

  // filteredAX = low_pass(2, dt, rawAX, rawAX1, rawAX2, aX1, aX2);
  // rawAX2 = rawAX1; rawAX1 = rawAX; aX2 = aX1; aX1 = filteredAX;
  // rawAX = filteredAX;

  if (this->shouldCalibrate && this->currentRound < this->calibrationRounds) {
    calibrationLedTime += dt;
    if (calibrationLedTime > 1) {
      calibrationIndication = !calibrationIndication;
      calibrationLedTime = 0;
      digitalWrite(9, calibrationIndication);
    }
    this->calibrate(
      rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ,
      this->currentRound == (this->calibrationRounds - 1)
    );
    this->currentRound ++;
    return false;
  }

  aX = rawAX - this->aErrorX;
  aY = rawAY - this->aErrorY;
  aZ = rawAZ;
  gX = rawGX - this->gErrorX;
  gY = rawGY - this->gErrorY;
  gZ = rawGZ - this->gErrorZ;

  this->setAngle(dt);
  
  return true;
}

void Gyro::calibrate(
  float saX,
  float saY,
  float saZ,
  float sgX,
  float sgY,
  float sgZ,
  bool lastSample = false
) {
    aErrorX += saX; 
    aErrorY += saY; 
    aErrorZ += saZ; 
    gErrorX += sgX; 
    gErrorY += sgY; 
    gErrorZ += sgZ;
    aAngleErrorX += atan(saY / sqrt(sq(saX) + sq(saZ)));
    aAngleErrorY += atan(-1 * saX / sqrt(sq(saY) + sq(saZ)));

    if (lastSample) {
      gErrorX /= calibrationRounds;
      gErrorY /= calibrationRounds;
      gErrorZ /= calibrationRounds;

      aErrorX /= calibrationRounds;
      aErrorY /= calibrationRounds;
      aErrorZ /= calibrationRounds;

      aAngleErrorX /= calibrationRounds;
      aAngleErrorY /= calibrationRounds;
      aAngleErrorZ /= calibrationRounds;

      digitalWrite(9, 1);
      delay(3000);
    }
}

void Gyro::setAngle(float dt) {
  // Detect accel angle
  float aAngleX 
    = (atan(aY / sqrt(sq(aX) + sq(aZ)))) * RAD_TO_DEG;
  float aAngleY 
    = (atan(-1 * aX / sqrt(sq(aY) + sq(aZ)))) * RAD_TO_DEG;
  float aAngleZ = (atan2(aY, aZ)) * RAD_TO_DEG;

  float nGPosX = 0.98 * (gPosX + gX * dt) - 0.02 * aAngleX;
  float nGPosY = 0.98 * (gPosY + gY * dt) - 0.02 * aAngleY;

  angularVelX = nGPosX - gPosX;
  angularVelY = nGPosY - gPosY;

  gPosX = nGPosX; 
  gPosY = nGPosY; 
  gPosZ = 0.98 * (gPosZ + gZ * dt) - 0.02 * aAngleZ;

  gpX += gX * dt; 
  gpY += gY * dt; 
  gpZ += gZ * dt;
}
