#include "utils.h"

float low_pass(
  float cutoff,
  float sample_time,
  float x0,
  float x1,
  float x2,
  float y1,
  float y2
) {
  float samp_rate = 1 / sample_time;
  float correction = 1.0;

  float corrected_cutoff = tan(PI * cutoff / samp_rate) / correction;

  float K1 = sqrt(2) * corrected_cutoff;
  float K2 = corrected_cutoff * corrected_cutoff;

  float a0 = K2 / (1 + K1 + K2);
  float a1 = 2 * a0;
  float a2 = a0;

  float K3 = a1 / K2;

  float b1 = -a1 + K3;
  float b2 = 1 - a1 - K3;

  return a0*x0 + a1*x1 + a2*x2 + b1*y1 + b2*y2;
}