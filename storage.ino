#include "storage.h"
#include <EEPROM.h>

void saveCalibration(
  float gErrorX,
  float gErrorY,
  float gErrorZ,
  float aErrorX,
  float aErrorY
) {
  EEPROM.put(0, gErrorX);
  EEPROM.put(1 * sizeof(float), gErrorY);
  EEPROM.put(2 * sizeof(float), gErrorZ);
  EEPROM.put(3 * sizeof(float), aErrorX);
  EEPROM.put(4 * sizeof(float), aErrorY);
}

void loadCalibration(
  float *gErrorX,
  float *gErrorY,
  float *gErrorZ,
  float *aErrorX,
  float *aErrorY
) {
  EEPROM.get(0, *gErrorX);
  EEPROM.get(1 * sizeof(float), *gErrorY);
  EEPROM.get(2 * sizeof(float), *gErrorZ);
  EEPROM.get(3 * sizeof(float), *aErrorX);
  EEPROM.get(4 * sizeof(float), *aErrorY);
}

void showCalibrationValues() {
  float gErrorX = 0;
  float gErrorY = 0;
  float gErrorZ = 0;
  float aErrorX = 0;
  float aErrorY = 0;

  EEPROM.get(0, gErrorX);
  EEPROM.get(1 * sizeof(float), gErrorY);
  EEPROM.get(2 * sizeof(float), gErrorZ);
  EEPROM.get(3 * sizeof(float), aErrorX);
  EEPROM.get(4 * sizeof(float), aErrorY);

  Serial.println(String(gErrorX) + " " + String(gErrorY) + " " + String(gErrorZ) + " " + String(aErrorX) + " " + String(aErrorY));
}