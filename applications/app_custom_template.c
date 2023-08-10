/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 1024);

// Private functions
static void pwm_callback(void);

static void terminal_set_airspeed(int argc, const char **argv);
static void terminal_set_torquescale(int argc, const char **argv);
static void terminal_set_randomcurrent(int argc, const char **argv);

static void terminal_set_appmode(int argc, const char **argv);
static void terminal_set_randomduty_min(int argc, const char **argv);
static void terminal_set_randomduty_max(int argc, const char **argv);

float mcpwm_foc_get_rpm_faster(void);
float mcpwm_foc_get_rpm_fast(void);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static float appmode = 0;
static float airspeed = 0;
static float torquescale = 1;
static float randomcurrent = 0;
static float randomduty_min = 0;
static float randomduty_max = 0;

#define MODE_OFF 0
#define MODE_PROPELLER 1
#define MODE_RANDOMDUTY 2

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	mc_interface_set_pwm_callback(pwm_callback);

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"airspeed",
			"set airspeed",
			"[airspeed]",
			terminal_set_airspeed);

	terminal_register_command_callback(
			"torquescale",
			"set torquescale",
			"[torquescale]",
			terminal_set_torquescale);

	terminal_register_command_callback(
			"randomcurrent",
			"set randomcurrent",
			"[randomcurrent]",
			terminal_set_randomcurrent);

	terminal_register_command_callback(
			"appmode",
			"set appmode",
			"[appmode]",
			terminal_set_appmode);

	terminal_register_command_callback(
			"randomduty_min",
			"set randomduty_min",
			"[randomduty_min]",
			terminal_set_randomduty_min);

	terminal_register_command_callback(
			"randomduty_max",
			"set randomduty_max",
			"[randomduty_max]",
			terminal_set_randomduty_min);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);
	terminal_unregister_callback(terminal_set_airspeed);
	terminal_unregister_callback(terminal_set_torquescale);
	terminal_unregister_callback(terminal_set_randomcurrent);
	terminal_unregister_callback(terminal_set_appmode);
	terminal_unregister_callback(terminal_set_randomduty_min);
	terminal_unregister_callback(terminal_set_randomduty_max);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static float rand_float(float min, float max) {
       return (((float)rand())/RAND_MAX * (max-min))+min;
}

static THD_FUNCTION(my_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Custom");

	is_running = true;


	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

		const volatile mc_configuration* conf_now = mc_interface_get_configuration();
		if (appmode == MODE_OFF) {
			mc_interface_set_current(0);
		} else if (appmode == MODE_PROPELLER) {
			float n_poles = conf_now->si_motor_poles / 2.0;
			float n = mcpwm_foc_get_rpm_fast()/60/n_poles;
			float V = airspeed;
			float D = .66;
			float rho = 1.225;
			float lambda = conf_now->foc_motor_flux_linkage;
			float J, CQ, torque, current;

			current = 0;

			if (n > 0) {
				J = 0;
				if (n > 1) {
					J = V/(n*D);
				}
				if (J < 0) {
					J = 0;
				}
				if (J > 1) {
					J = 1;
				}

				CQ = 0.0297 + 0.0241*J + -0.0331*J*J + -0.0316*J*J*J;
				torque = torquescale*CQ * rho * n*n * D*D*D*D*D;
				current = -torque/(1.5 * lambda * n_poles);
			}

			static float current_modifier = 0;
			static systime_t last_change_time = 0;
			uint32_t t_elapsed_ms = ST2MS(chVTTimeElapsedSinceX(last_change_time));
			if (t_elapsed_ms > 300) {
					last_change_time = chVTGetSystemTimeX();
					current_modifier = rand_float(-randomcurrent,randomcurrent);
			}
			mc_interface_set_current(current+current_modifier);
		} else if (appmode == MODE_RANDOMDUTY) {
			mc_interface_set_duty(rand_float(randomduty_min,randomduty_max));
		}

		chThdSleepMicroseconds(100);
	}
}

static void pwm_callback(void) {

}

// Callback function for the terminal command with arguments.
static void terminal_set_torquescale(int argc, const char **argv) {
	if (argc == 2) {
		float cmd_torquescale = -1;
		sscanf(argv[1], "%f", &cmd_torquescale);

		if (cmd_torquescale >= 0) {
			torquescale = cmd_torquescale;
			commands_printf("torquescale set to %f", torquescale);
		} else {
			commands_printf("torquescale must be a nonnegative number");
		}
	} else {
		commands_printf("torquescale = %f\n", torquescale);
	}
}


// Callback function for the terminal command with arguments.
static void terminal_set_airspeed(int argc, const char **argv) {
	if (argc == 2) {
		float cmd_airspeed = -1;
		sscanf(argv[1], "%f", &cmd_airspeed);

		if (cmd_airspeed >= 0) {
			airspeed = cmd_airspeed;
			commands_printf("Airspeed set to %f", airspeed);
		} else {
			commands_printf("Airspeed must be a nonnegative number");
		}
	} else {
		commands_printf("airspeed = %f\n", airspeed);
	}
}


// Callback function for the terminal command with arguments.
static void terminal_set_randomcurrent(int argc, const char **argv) {
	if (argc == 2) {
		float cmd_randomcurrent = -1;
		sscanf(argv[1], "%f", &cmd_randomcurrent);

		if (cmd_randomcurrent >= 0) {
			randomcurrent = cmd_randomcurrent;
			commands_printf("randomcurrent set to %f", randomcurrent);
		} else {
			commands_printf("randomcurrent must be a nonnegative number");
		}
	} else {
		commands_printf("randomcurrent = %f\n", randomcurrent);
	}
}


// Callback function for the terminal command with arguments.
static void terminal_set_appmode(int argc, const char **argv) {
	if (argc == 2) {
		float cmd_appmode = -1;
		sscanf(argv[1], "%f", &cmd_appmode);

		if (cmd_appmode >= 0) {
			appmode = cmd_appmode;
			commands_printf("appmode set to %f", appmode);
		} else {
			commands_printf("appmode must be a nonnegative number");
		}
	} else {
		commands_printf("appmode = %f\n", appmode);
	}
}

// Callback function for the terminal command with arguments.
static void terminal_set_randomduty_min(int argc, const char **argv) {
	if (argc == 2) {
		float cmd_randomduty_min = -1;
		sscanf(argv[1], "%f", &cmd_randomduty_min);

		if (cmd_randomduty_min >= 0) {
			randomduty_min = cmd_randomduty_min;
			commands_printf("randomduty_min set to %f", randomduty_min);
		} else {
			commands_printf("randomduty_min must be a nonnegative number");
		}
	} else {
		commands_printf("randomduty_min = %f\n", randomduty_min);
	}
}

// Callback function for the terminal command with arguments.
static void terminal_set_randomduty_max(int argc, const char **argv) {
	if (argc == 2) {
		float cmd_randomduty_max = -1;
		sscanf(argv[1], "%f", &cmd_randomduty_max);

		if (cmd_randomduty_max >= 0) {
			randomduty_max = cmd_randomduty_max;
			commands_printf("randomduty_max set to %f", randomduty_max);
		} else {
			commands_printf("randomduty_max must be a nonnegative number");
		}
	} else {
		commands_printf("randomduty_max = %f\n", randomduty_max);
	}
}
