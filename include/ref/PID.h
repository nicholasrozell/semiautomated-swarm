#ifndef _PID_H
#define _PID_H

#include <iostream>
#include <cmath>
#include <stack>
#include <ctime>
#include "../global/global.h"

using namespace std;

// Structures
// Structures
typedef struct PIDgains{

	float P;
	float I;
	float D;

}PIDgains;

typedef struct pid_error{

	float p;
	float i;
	float d;
	float last;

}pid_error;

// Classes
class pid{
private:

	//Gains, Limits & Error Terms
	PIDgains K;
	float limit;
	bool flag;
	float u;
	float u_unsat;
	float tau;
	//Converts the digital readout from  the sensor to a physical measurement with units
	float sat(float in, float limit);

public:
	//Members
	pid_error e;
	float Ts;

	// Timer for Ts
	std::stack<clock_t> tictoc_stack;
	void toc();
	void tic();

	//Constructor function for the C++ class
	pid();

	void init(float kp, float ki, float kd, float limit_in, float tau_in, float Ts_in);

	// Compute the PID Loop
	float controller_output(float y_c, float y, bool flag);
	float controller_output(float error, bool flag);

	// Gets
	float get_proportional_error();
	float get_integral_error();
	float get_derivative_error();
	void print_error(string pid_name);
	void print_gains_error(string pid_name);

	//Function destuctor
	virtual ~pid();

};

#endif
