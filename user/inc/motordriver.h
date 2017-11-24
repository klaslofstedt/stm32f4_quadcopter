#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

void motordriver_init(void);
void motordriver_set1(uint32_t i, uint32_t m_delay);
void set_channels_1(float duty1, float duty2, float duty3);
void set_channels_2(float duty1, float duty2, float duty3);
void set_channels_3(float duty1, float duty2, float duty3);

#endif