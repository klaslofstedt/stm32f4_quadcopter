#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

float filter_transition(float data1, float data2, float damping);
float filter_lowpass(float update, float data1, float damping);
void filter_complementary(float *update, float data1, float data2, float dt, float damping);

#endif