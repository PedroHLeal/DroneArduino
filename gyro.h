#ifndef GYRO
#define GYRO

#define SHOULD_CALIBRATE_GYRO 1
class Gyro {
  public:
    Gyro(bool shouldCalibrate = false);
    bool updateData(float dt);
    float
      aX = 0, aY = 0, aZ = 0,
      gX = 0, gY = 0, gZ = 0,
      gPosX = 0, gPosY = 0, gPosZ = 0,
      gpX = 0, gpY = 0, gpZ = 0,
      angularVelX = 0, angularVelY = 0;
  private:
    int calibrationRounds = 3000, currentRound = 0;
    int samplesToIgnore = 3000, ignoredSamples = 0;
    bool shouldCalibrate = false;
    int calibrationIndication = 0;
    float calibrationLedTime = 0;
    float
      rawAX = 0, rawAY = 0, rawAZ = 0,
      rawAX1 = 0, rawAX2 = 0, aX1 = 0, aX2 = 0, filteredAX = 0,
      rawGX = 0, rawGY = 0, rawGZ = 0,
      gErrorX = 0, gErrorY = 0, gErrorZ = 0,
      aErrorX = 0, aErrorY = 0, aErrorZ = 0,
      aAngleErrorX = 0, aAngleErrorY = 0, aAngleErrorZ = 0,
      tmp = 0;
    void setPosition(float dt);
    void calibrate(
      float aX,
      float aY,
      float aZ,
      float gX,
      float gY,
      float gZ,
      bool lastSample = false
    );
    void setAngle(float dt);
};

class GyroSingleton {
  public:
    static Gyro* getGyro() {
      if (g == NULL) {
        g = new Gyro(SHOULD_CALIBRATE_GYRO);
      }
      return g;
    }
  private:
    static Gyro *g;
};

Gyro* GyroSingleton::g = NULL;

#endif