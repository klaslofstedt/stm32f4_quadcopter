#include "filter.h"


float filter_transition(float data1, float data2, float damping)
{
    return (data1 * (1.0f - damping) + data2 * damping);
}

float filter_lowpass(float update, float data1, float damping)
{
    return (update * damping + (1.0f - damping) * data1);
}

void filter_complementary(float* update, float data1, float data2, float dt, float damping)
{
    
}