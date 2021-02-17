/*

 * Control.cpp takes in values for global.c and
 * determines servo positions based on those
 * values.
 *
 *
 * THREADED AND DETACHED
 *
 * Written by
 * Last Modified by Dane Johnson on 1/16/14
 *
 */

/*
 * System includes
 */
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stropts.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <armadillo>

/*
 * File includes
 */
#include "../uav_state_estimation/uavStates.h"
#include "../global/global.h"
#include "../mavlink/v1.0/common/mavlink.h"
#include "../mavlink/v1.0/common/common.h"

#define _USE_MATH_DEFINES

/*
 * Function Prototypes
 */
void *control(void *arg);
void get_commanded_attitude();
int check_waypoints_home(std::vector<waypoint>*, home_location*);
void toggle_gpio(unsigned int counter);

// Namespaces
using namespace std;
using namespace arma;

/********************************* Main Thread ****************************************/

void *control(void *arg) {

	/*Declaration of variables*/
	float dt = 0; //40 Hz

	if(SIMULATION_MODE == true || HIL_MODE == true){
		dt = .0238; //42 Hz for XPlane freq.
	} else if(SIMULATION_MODE == false && HIL_MODE == false){
		dt = .0156; //64 Hz for sensors
	} else {
		dt = .025; //~40 Hz for all other modes
	}

	unsigned int counter;
	unsigned long temp_sensor_update = 0;
	int mode_check;
	int mode_toggle_val = mode.get_toggle_val();
	/* Declaration of objects  */
	guidance outterLoop;
	//COMMENTED OUT PID FOR CALLING MRAC
//	pid phi_pid;
//	pid theta_pid;
//	pid psi_pid;
	GPMRAC phi_mrac
	GPMRAC theta_mrac
	GPMRAC psi_mrac
	/*	Ensure that the system is uncontrollable until
	 *  certain	conditions are met to prevent take-off*/
	check_waypoints_home(&WPList, &home);

	/* Object initialization */
//	phi_pid.init(K_PHI_P, K_PHI_I, K_PHI_D, LIMIT_AIL, 0, dt); // Initialize the roll pid for vehicle attitude
//	theta_pid.init(K_THETA_P, K_THETA_I, K_THETA_D, LIMIT_ELEV, 0, dt); // Initialize the pitch pid for vehicle attitude
//	psi_pid.init(K_PSI_P, K_PSI_I, K_PSI_D, LIMIT_RUDDER, 0, dt); // Initialize the roll pid for vehicle attitude
	phi_mrac(omega_phi, eta_phi, K1_phi, K2_phi,f_hat_phi, b_hat_phi, GPParams_phi,frequency); 
	theta_mrac(omega_theta, eta_theta, K1_theta, K2_theta,f_hat_theta, b_hat_theta, GPParams_theta,frequency); 
	psi_mrac(omega_psi, eta_psi, K1_psi, K2_psi,f_hat_psi, b_hat_psi, GPParams_psi,frequency); 
	outterLoop.init(dt);					// Initialize the guidance loops

	/* Control thread loop */
	while (true) {

		UAV.systemStatus = MAV_STATE_ACTIVE;  // current UAV state

		/* Zero out the PIDs to prevent integrator wind-up
		 * and snappy manuevers - a true boolean value zeros
		 * integrator and differentiator. See pid class for more
		 * details */
		phi_pid.controller_output(0, true);
		theta_pid.controller_output(0, true);

		/* Provided new gains or waypoints, reinitialize the gains
		 * and guidance. */
		if (new_params_loaded) {
//			/* Show new gains for testing purposes - delete later */
//			cout << "K_PHI_P = " << K_PHI_P << " K_PHI_I = " << K_PHI_I
//					<< " K_PHI_D = " << K_PHI_D << endl;
//			cout << "K_THETA_P = " << K_THETA_P << " K_THETA_I = " << K_THETA_I
//					<< " K_THETA_D = " << K_THETA_D << endl;
//			cout << "K_PSI_P = " << K_PSI_P << " K_PSI_I = " << K_PSI_I
//					<< " K_PSI_D = " << K_PSI_D << endl;
//			cout << "K_ALT_P = " << K_ALT_P << " K_ALT_I = " << K_ALT_I
//					<< " K_ALT_D = " << K_ALT_D << endl;
//			cout << "K_XTRACK_P = " << K_XTRACK_P << " K_XTRACK_I = "
//					<< K_XTRACK_I << " K_XTRACK_D = " << K_XTRACK_D << endl;
//			cout << "K_THROT_P = " << K_THROT_P << " K_THROT_I = " << K_THROT_I
//					<< " K_THROT_D = " << K_THROT_D << endl;
			// COMMENTED OUT PID FOR CALLING MRAC
			// Reinitialize the pids
//			phi_pid.init(K_PHI_P, K_PHI_I, K_PHI_D, LIMIT_AIL, 1, dt); // Initialize the roll pid for vehicle attitude
//			theta_pid.init(K_THETA_P, K_THETA_I, K_THETA_D, LIMIT_ELEV, 2, dt); // Initialize the pitch pid for vehicle attitude
//			psi_pid.init(K_PSI_P, K_PSI_I, K_PSI_D, LIMIT_RUDDER, 1, dt); // Initialize the roll pid for vehicle attitude
			phi_mrac(omega_phi, eta_phi, K1_phi, K2_phi,f_hat_phi, b_hat_phi, GPParams_phi,frequency); 
			theta_mrac(omega_theta, eta_theta, K1_theta, K2_theta,f_hat_theta, b_hat_theta, GPParams_theta,frequency); 
			psi_mrac(omega_psi, eta_psi, K1_psi, K2_psi,f_hat_psi, b_hat_psi, GPParams_psi,frequency); 
			outterLoop.init(dt);				// Initialize the guidance loops
			new_params_loaded = false; // ends the process of loading new params
		}

		/* Test Mode Switch - Change to the mavlink	setting */
		if (mode.get_toggle_val() == 1) {
			UAV.systemMode = MAV_MODE_MANUAL_ARMED;  // current UAV control mode
			mode_check = 1;
		} else if (mode.get_scaled_val() <= .5
				&& mode.get_scaled_val() >= -.5) {
			UAV.systemMode = MAV_MODE_STABILIZE_ARMED; // current UAV control mode
			mode_check = 0;
		} else if (mode.get_scaled_val() < -.5) {
			UAV.systemMode = MAV_MODE_AUTO_ARMED;
			mode_check = -1;
		}

		while (mode_check == mode_toggle_val) { // If mode is switched, break and re-initialize loops

			/* Delay for the pid sampling time */
//			usleep((int) 1e6 * dt);
			mode_toggle_val = mode.get_toggle_val();

			uav_state_propagation();
			/*Propagate PID commands - this keeps the timing of
			 the loops consistent */
			aileron.cmd = phi_pid.controller_output(uav_error.phi, 0);
			elevator.cmd = theta_pid.controller_output(cmd.euler_angle.theta,
					uav.euler_angle.theta, 0);

			// Test
//			cout << " toggle_val: " << mode.get_toggle_val() << " scaled_val: "
//					<< mode.get_scaled_val() << endl;

			/* Manual mode */
			if (mode_toggle_val == 1) { // Test to see if the RX switch is in manual mode
				aileron.output_pwm();
				elevator.output_pwm();
				rudder.output_pwm();
				throttle.output_pwm();
				flaps.output_pwm();
				aux1.output_pwm();
				mode.output_pwm();
				trigger.output_pwm();
				// Temporary code - remove later
				if (abs(trigger.pwm_zero - trigger.get_pwm()) > 10) {
					outterLoop.reset_to_runway();
					//cout << "Reset to runway" << endl;
				}
//				cout << " Outputting PWM: " << mode.get_toggle_val() << endl;
			}

			/* Attitude hold mode */
			else if (mode_toggle_val == 0) { // Test to see if the RX switch is in attitude hold mode ()

				/*Check for tuning commands - This is temporary*/
				if (aux1.get_scaled_val() < -.5) { // Checks AUX1 for the commanded attitude
					cmd.euler_angle.theta = .174532925; // 10 deg
					cmd.euler_angle.phi = 0;
				} else if (aux1.get_scaled_val() <= .5
						&& aux1.get_scaled_val() >= -.5) {
					cmd.euler_angle.theta = 0;		// Set euler angle commands
					cmd.euler_angle.phi = 0;	// to zero for approximately SLF
				} else {
					cmd.euler_angle.theta = 0;		// Set euler angle commands
					cmd.euler_angle.phi = .34;		// ~20 deg
				}

				if(temp_sensor_update < sensorUpdate){
					temp_sensor_update = sensorUpdate;
					/* Propagate all relevant states for control */
					uav_error = compute_uav_error(uav, cmd);

					/*Output the commanded value from pid to the servo driver*/
					aileron.output_cmd(true);
					elevator.output_cmd(false);
				}

				/*Output the RX values for the remaining controls*/
				rudder.output_pwm();
				throttle.output_pwm();
				flaps.output_pwm();
				flaps.output_pwm();
				aux1.output_pwm();
				mode.output_pwm();
				trigger.output_pwm();

				//while(temp_sensor_update == sensorUpdate){;}

//				cout << " Attitude Hold: " << mode.get_toggle_val() << endl;

			} // end stabilize mode

			/* Guidance mode */
			else if (mode_toggle_val == -1) {
				//Don't act on data till it has been updated
				if(temp_sensor_update < sensorUpdate){
					temp_sensor_update = sensorUpdate;

					/* Propagate all relevant states for control */
					uav_error = compute_uav_error(uav, cmd);

					/* Outer loop guidance - populates euler commands
					 *  with guidance values instead of	preset values*/
					outterLoop.update_attitude_cmd();

					aileron.output_cmd(true);
					elevator.output_cmd(false);

					rudder.cmd = nosewheel_cmd;
					rudder.output_cmd(false);

					    throttle.cmd = throttle_cmd;



					throttle.output_cmd(false);

					flaps.cmd = 0;
					flaps.output_cmd(false);
				}
//				cout << " Guidance: " << mode.get_toggle_val() << endl;

			} // end if
//			counter++;
//			toggle_gpio(counter);

		} // end while

	} // end while loop (main loop)

}  // end control thread

/****************************** Auxiliary Functions *************************************/

int check_waypoints_home(std::vector<waypoint> *flightPlan,
		home_location *ned_home) {

	//cout << ned_home->lat << "\t" << ned_home->lon << endl;

	// Let the user know that home is not set
	string_convert << CONSOLE_ENTRY_NUMBER++
			<< " - Controls locked until home is set!";
	sendToConsole();
	/* Ensure that the waypoints are uploaded */
	while (ned_home->lat == 0) {
		usleep(2e6);
//		// Let the user know that home is not set
//		string_convert << CONSOLE_ENTRY_NUMBER
//				<< " - Controls locked until home is set!";
//		sendToConsole();
		usleep(5e6);  // No need to overload the console
	}

	// Let the user know that waypoints are not set
	string_convert << CONSOLE_ENTRY_NUMBER++
			<< " - Please upload waypoints!";
	sendToConsole();
	/* Ensure that the waypoints are uploaded */
	while (flightPlan->empty()) {
//		// Let the user know that waypoints are not set
//		string_convert << CONSOLE_ENTRY_NUMBER + 1
//				<< " - Please upload waypoints!";
//		sendToConsole();
		usleep(5e6);  // No need to overload the console
	}

	while (gcs_comms.inReceivingnavigation) {
		string_convert << CONSOLE_ENTRY_NUMBER + 1 << " - Receiving waypoints";
		sendToConsole();
		usleep(5e6);
	}
	string_convert << CONSOLE_ENTRY_NUMBER + 2 << " - Waypoints uploaded";
	sendToConsole();

}

void toggle_gpio(unsigned int counter) {
	if(HIL_MODE == false){
		if (counter % 2 == 0) {
			system("echo 1 > /sys/class/gpio/gpio45/value");
		} else {
			system("echo 0 > /sys/class/gpio/gpio45/value");
		}
	}
}
