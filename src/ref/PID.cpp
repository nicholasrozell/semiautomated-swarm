/*%%%%%%%%%%%%%%%%%%% PID Controller Class %%%%%%%%%%%%%%%%%%%%%%%%%%%
 % Author: Jacob Stockton
 % Date: 2/19/14
 % Purpose: OSU Stabilis Autopilot Navigation and Guidance.
 %
 % Ref 1:  Small Unmanned Aircraft: Theory and Practice. Beard, Randall. Mclain, Timothy
 %		  Refer to pg 116-117
 %
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "../global/global.h"

using namespace std;

// Constructor
pid::pid() {
}
;

void pid::init(float kp, float ki, float kd, float limit_in, float tau_in,
		float Ts_in) {

	// Assigns gains locally
	K.P = kp;
	K.I = ki;
	K.D = kd;

	e.p = 0;
	e.i = 0;
	e.d = 0;
	e.last = 0;

	Ts = Ts_in;
	limit = limit_in;
	tau = tau_in;

}

// http://stackoverflow.com/questions/13485266/how-to-have-matlab-tic-toc-in-c
void pid::tic() {
	tictoc_stack.push(clock());
}

void pid::toc() {
	std::cout << "Time elapsed: "
			<< ((double) (clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
			<< std::endl;
	tictoc_stack.pop();
}

float pid::sat(float in, float limit) {

	// Check to see if the
	if (abs(in) > limit) {
		return copysign(limit, in);
	}

	return in;
}

float pid::controller_output(float y_c, float y, bool flag) {

	// If the pid has been flagged persistent variables
	if (flag == true) {
		e.i = 0;
		e.d = 0;
		e.last = 0;
	}

	// Calculate errors
	e.p = y_c - y;						// Proportional error
	e.i = e.i + Ts / 2 * (e.p + e.last);	// integrator error
	e.d = (2 * tau - Ts) / (2 * tau + Ts) * e.d
			+ (2 / (2 * tau + Ts)) * (e.p - e.last);	// Derivative error
	e.last = e.p;			// Store the current value for the next iteration

	// Calculate the raw output of the pid
	u_unsat = K.P * e.p + K.I * e.i + K.D * e.d;
	// Check for saturation
	u = sat(u_unsat, limit);

	//Check to see if anti-windup measures are needed
	if (K.I != 0) {
		e.i = e.i + Ts / K.I * (u - u_unsat);
	}

	return u;

}

float pid::controller_output(float error, bool flag) {

	// If the pid has been flagged persistent variables
	if (flag == 1) {
		e.i = 0;
		e.d = 0;
		e.last = 0;
	}

	// Calculate errors
	e.p = error;						// Proportional error
	e.i = e.i + Ts / 2 * (e.p + e.last);	//integrator error
	e.d = (2 * tau - Ts) / (2 * tau + Ts) * e.d
			+ (2 / (2 * tau + Ts)) * (e.p - e.last);	// Derivative error
	e.last = e.p;			// Store the current value for the next iteration

//	cout << "Testing differentiator" << K.D*e.d << endl;

	// Calculate the raw output of the pid
	u_unsat = K.P * e.p + K.I * e.i + K.D * e.d;

	// Check for saturation
	u = sat(u_unsat, limit);

	//Check to see if anti-windup measures are needed
	if (K.I != 0) {
		e.i = e.i + Ts / K.I * (u - u_unsat);
	}

	return u;

}

float pid::get_proportional_error() {
	return e.p;
}

float pid::get_integral_error() {
	return e.i;
}

float pid::get_derivative_error() {
	return e.d;
}

void pid::print_error(string pid_name) {
	cout << pid_name << " : " << "e.p " << e.p << "\t e.i " << e.i << "\t e.d"
			<< e.d << endl;
}

void pid::print_gains_error(string pid_name) {
	cout << pid_name << " : " << "K_p*e_p " << K.P * e.p << "\t K_i*e_i "
			<< K.I * e.i << "\t K_d*e_d" << K.D * e.d << endl;
}
// Destructor
pid::~pid() {
}
