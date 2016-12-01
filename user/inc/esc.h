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
} esc_t;

void esc_set_speed(esc_t *esc);
void esc_init(esc_t *esc, uint16_t pin);
void esc_check_bondaries(esc_t *esc);

#endif
