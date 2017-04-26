#include "filter.h"


void filter_transition(float *update, float data1, float data2, float damping)
{
    *update = data1 * (1.0f - damping) + data2 * damping;
}

void filter_lowpass(float* update, float data1, float damping)
{
    
}

void filter_complementary(float* update, float data1, float data2, float dt, float damping)
{
    
}