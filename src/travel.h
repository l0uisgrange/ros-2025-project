#ifndef TRAVEL_H
#define TRAVEL_H

#define GOAL_TILT_Y           	0.0f
#define TILT_KP              	500.0f 	// Proportional gain for Y-tilt
#define TILT_KI               	1.0f   	// Integral gain for Y-tilt
#define TILT_ERROR_THRESHOLD 	0.5f 
#define TILT_MAX_SUM_ERROR    	50.0f  	// Anti-windup limit

#define GOAL_TILT_X             0.0f
#define ROTATION_TILT_THRESHOLD 0.8f
#define ROTATION_TILT_COEFF     70.0f   // Proportional gain for rotation speed correction

#define AVERAGING_SIZE 30

void travel_thread_start(void);

void set_enabled_motors(bool enable);
void toggle_enabled_motors(void);

void set_speed(int16_t left, int16_t right);
void set_pos(int16_t left, int16_t right);
float get_imu_average(void);
void add_imu_average(float value);
void reset_imu_averaging(void);

#endif