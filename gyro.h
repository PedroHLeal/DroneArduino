#ifndef GYRO
#define GYRO

#define SHOULD_CALIBRATE_GYRO 1
class Gyro {
  public:
    void readCalibration();
    bool calibrate();
    Gyro(bool shouldCalibrate = false);
    bool updateData(float dt);
    float
      aX = 0, aY = 0, aZ = 0,
      gX = 0, gY = 0, gZ = 0,
      gPosX = 0, gPosY = 0, gPosZ = 0,
      posX = 0, posY = 0;
  private:
    int calibrationRounds = 3000, currentRound = 0;
    int samplesToIgnore = 3000, ignoredSamples = 0;
    bool shouldCalibrate = false;
    int calibrationIndication = 0;
    float calibrationLedTime = 0;
    float
      rawAX = 0, rawAY = 0, rawAZ = 0,
      rawGX = 0, rawGY = 0, rawGZ = 0,
      gErrorX = 0, gErrorY = 0, gErrorZ = 0,
      aErrorX = 0, aErrorY = 0, aErrorZ = 0,
      tmp = 0;
    void setAngle(float dt);
    void readRawValues();
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