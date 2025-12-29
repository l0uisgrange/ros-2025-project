#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <leds.h>
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include "motors.h"
#include <chprintf.h>
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "msgbus/messagebus.h"

#include "main.h"
#include "travel.h"
#include "detection.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) {
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void) {
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void) {
    halInit();
    chSysInit();
    mpu_init();

    // Starts the serial communication
    serial_start();
	messagebus_init(&bus, &bus_lock, &bus_condvar);

    chThdSleepMilliseconds(100);

	// Shows LED for configuration steps
	set_rgb_led(LED2, 0, 255, 0);
	set_rgb_led(LED4, 0, 255, 0);
	set_rgb_led(LED6, 0, 255, 0);
	set_rgb_led(LED8, 0, 255, 0);
	set_led(LED2, true);

	// Configures and enables the IMU and proximity (IR) sensors
	chprintf((BaseSequentialStream *)&SD3, "[INIT 1/6] Starting sensors\r\n");
	imu_start();
	proximity_start();
	set_led(LED4, true);

    // Waits 2s to be sure the e-puck is in a stable position
	chprintf((BaseSequentialStream *)&SD3, "[INIT 2/6] Waiting stabilization\r\n");
    chThdSleepMilliseconds(2000);
	set_led(LED6, true);

	// IMU and proximity sensors Calibration
	chprintf((BaseSequentialStream *)&SD3, "[INIT 3/6] Calibrating sensors\r\n");
    calibrate_acc();
	calibrate_ir();
	set_led(LED8, true);

	// Inits and enables the motors
	chprintf((BaseSequentialStream *)&SD3, "[INIT 4/6] Starting motors\r\n");
	motors_init();
	set_enabled_motors(true);

	// Starts the threads
	chprintf((BaseSequentialStream *)&SD3, "[INIT 5/6] Starting regulators\r\n");
	travel_thread_start();
	wall_detection_start();

	chprintf((BaseSequentialStream *)&SD3, "[INIT 6/6] Finished initialisation\r\n");
	set_rgb_led(LED4, 0, 255, 0);
	clear_leds();
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}
