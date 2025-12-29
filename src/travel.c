#include <ch.h>
#include <chprintf.h>
#include <hal.h>
#include <math.h>
#include <motors.h>
#include <sensors/imu.h>
#include <usbcfg.h>

#include "detection.h"
#include "main.h"
#include "travel.h"

static bool enabled_motors = false;
static float tilt_sum_error = 0;
static float tilt_y_history[AVERAGING_SIZE];
static uint8_t history_index = 0;

void reset_imu_averaging(void) {
    for (int8_t i = 0; i < AVERAGING_SIZE; i++) {
        tilt_y_history[i] = 0;
    }
    history_index = 0;
}

void set_speed(int16_t left, int16_t right) {
    left_motor_set_speed(left);
    right_motor_set_speed(right);
}

void set_pos(int16_t left, int16_t right) {
    left_motor_set_pos(left);
    right_motor_set_pos(right);
}

void add_imu_average(float value) {
    tilt_y_history[history_index] = value;

    if (history_index < AVERAGING_SIZE) {
        history_index++;
    } else {
        history_index = 0;
    }
}

float get_imu_average() {
    float sum = 0;

    for (uint8_t i = 0; i < AVERAGING_SIZE; i++) {
        sum += tilt_y_history[i];
    }
    return sum / AVERAGING_SIZE;
}

int16_t pi_regulator(float current_tilt, float goal_tilt) {
    float error = 0;
    float speed = 0;

    error = current_tilt - goal_tilt;

    if (fabs(error) < TILT_ERROR_THRESHOLD) {
        return 0;
    }

    tilt_sum_error += error;

    // Anti-windup
    if (tilt_sum_error > TILT_MAX_SUM_ERROR) {
        tilt_sum_error = TILT_MAX_SUM_ERROR;
    } else if (tilt_sum_error < -TILT_MAX_SUM_ERROR) {
        tilt_sum_error = -TILT_MAX_SUM_ERROR;
    }

    speed = TILT_KP * error + TILT_KI * tilt_sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waTravelThead, 256);
static THD_FUNCTION(TravelThread, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    float speed_correction = 0;

    float current_tilt_y = 0;
    float current_tilt_x = 0;

    while (true) {
        time = chVTGetSystemTime();

        if (in_bounce_procedure() && !achieved_bounce_distance()) {
            set_speed(-800, -800);
        } else if (in_turn_procedure() && !achieved_turn_distance()) {
            set_speed(800, -800);
        } else {
            // === Forward/Backward Control ===

            current_tilt_y = get_acceleration(Y_AXIS);

            // IMU values smoothing
            add_imu_average(current_tilt_y);
            float average_tilt_y = get_imu_average();
            speed = pi_regulator(average_tilt_y, GOAL_TILT_Y);

            // === Rotation Control ===

            current_tilt_x = get_acceleration(X_AXIS);

            if (fabs(current_tilt_x - GOAL_TILT_X) > ROTATION_TILT_THRESHOLD) {
                speed_correction = ROTATION_TILT_COEFF * (current_tilt_x - GOAL_TILT_X);
            } else {
                speed_correction = 0;
            }

            if (enabled_motors) {
                set_speed(speed + (int16_t)speed_correction, speed - (int16_t)speed_correction);
            } else {
                set_speed(0, 0);
            }
        }

        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void travel_thread_start(void) {
    tilt_sum_error = 0;
    chThdCreateStatic(waTravelThead, sizeof(waTravelThead), NORMALPRIO, TravelThread, NULL);
}

void set_enabled_motors(bool enable) {
    enabled_motors = enable;
    if (!enable) {
        set_speed(0, 0);
    }
}

void toggle_enabled_motors() { set_enabled_motors(!enabled_motors); }
