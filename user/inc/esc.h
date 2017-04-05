#ifndef ESC_h
#define ESC_h

typedef struct{
	uint16_t init_low;
	uint16_t init_high;
	uint16_t run_min;
	uint16_t run_max;
	uint16_t lift_quad_min;
	uint8_t pin_number;
	float speed; // from 0 to 1, with a resolution of ~14 bits (14280 values)
    float speed_max;
    float speed_min;
} esc_t;

void esc_set_speed(esc_t *esc, float speed);
void esc_init(esc_t* esc);


#endif
