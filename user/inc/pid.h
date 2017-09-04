#ifndef PID_h
#define PID_h

/*#define Kp 0.0
#define Ki 0.0
#define Kd 0.0*/

typedef struct{
	float input;
    float last_input;
    float rate;
    float rate_calc;
	float setpoint;
    float boundary_max;
    float boundary_min;
	float i_term;
    float last_error;
	float output;
    float k_p;
    float k_i;
    float k_d;
}pid_data_t;

void pid_calc(pid_data_t* pid, unsigned long dt);

#endif
