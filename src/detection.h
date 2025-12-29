#ifndef DETECTION_H
#define DETECTION_H

typedef enum {
    IR1 = 0,
    IR2,
    IR3,
	IR4,
   	IR5,
	IR6,
	IR7,
	IR8,
} ir_direction_t;

#define NSTEP_BOUNCING              1000 // Motor steps distance to complete first bounce
#define BOUNCING_DAMPING_FACTOR     0.4 
#define NBOUNCES                    7
#define BOUNCING_DISTANCE           2000 // Distance wall to IR [IR unit]

#define PI                          3.1415926536f
#define NSTEP_ONE_TURN              1000
#define WHEEL_DISTANCE              5.35f // [cm]
#define PERIMETER_PUCK              PI * WHEEL_DISTANCE // [cm]
#define WHEEL_PERIMETER             13 // [cm]
#define HALF_TURN_DISTANCE          PERIMETER_PUCK / 2.0f * NSTEP_ONE_TURN / WHEEL_PERIMETER // [steps]

void wall_detection_start(void);
bool in_bounce_procedure(void);
bool in_turn_procedure(void);
bool is_against_wall(void);
bool achieved_bounce_distance(void);
bool achieved_turn_distance(void);

#endif