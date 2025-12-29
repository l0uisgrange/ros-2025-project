
#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <motors.h>

#include "travel.h"
#include "detection.h"
#include "main.h"

struct State {
    bool bouncing_enabled;
    bool turning_enabled;
    int bouncing_counter;
};

static struct State state = {
    .bouncing_enabled = false,
    .turning_enabled = false,
    .bouncing_counter = 1
};

static THD_WORKING_AREA(waWallDetection, 256);
static THD_FUNCTION(WallDetection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(true) {
        time = chVTGetSystemTime();

        if (is_against_wall() && !state.bouncing_enabled && !state.turning_enabled) {
            state.bouncing_enabled = true;
            if (state.bouncing_counter < NBOUNCES) {
                state.bouncing_counter++;
            } else {
                state.bouncing_enabled = false;
                state.turning_enabled = true;
                state.bouncing_counter = 0;
            }
            set_pos(0, 0);
        }

        if (achieved_turn_distance() && state.turning_enabled) {
            state.turning_enabled = false;
            reset_imu_averaging();
        }

        if (state.bouncing_enabled) {
            set_body_led(true);
        } else {
            set_body_led(false);
        }

        for (int led = 0; led < LED8; led++) {
            set_led(led, state.bouncing_counter == led);
        }

        chThdSleepUntilWindowed(time, time);
    }
}

void wall_detection_start(void) {
	chThdCreateStatic(waWallDetection, sizeof(waWallDetection), NORMALPRIO, WallDetection, NULL);
}

bool in_turn_procedure(void) {
    return state.turning_enabled;
}

bool in_bounce_procedure(void) {
    return state.bouncing_enabled;
}

bool achieved_turn_distance(void) {
    return abs(right_motor_get_pos()) > HALF_TURN_DISTANCE && abs(left_motor_get_pos()) > HALF_TURN_DISTANCE;
}

bool achieved_bounce_distance(void) {
    bool completed = abs(right_motor_get_pos()) >  NSTEP_BOUNCING * exp(- BOUNCING_DAMPING_FACTOR * state.bouncing_counter);
    if (completed) {
        state.bouncing_enabled = false;
    }
    return completed;
}

bool is_against_wall() {
	if (get_calibrated_prox(IR8) > BOUNCING_DISTANCE || get_calibrated_prox(IR1) > BOUNCING_DISTANCE) {
		return true;
	}
	return false;
}
