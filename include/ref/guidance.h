/*%%%%%%%%%%%%%%%%%%% GP-MRAC Quadrotor Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Jacob Stockton
% Date: 2/19/14
% Purpose: OSU Stabilis Autopilot Navigation and Guidance.
%
% Ref 1:  Unmanned Rotorcraft Systems. Guowei Cai, Ben M. Chen, Tong Heng Lee
% Ref 2:  Small Unmanned Aircraft: Theory and Practice. Beard, Randall. Mclain, Timothy
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#ifndef _GUIDANCE_H
#define _GUIDANCE_H

//Standard library
#include <armadillo>

//Local includes
#include "../global/global.h"
#include "control.h"
#include "PID.h"
#include "MRAC.h"// include libraries of GPMRAC

#define _USE_MATH_DEFINES

// Axes
#define ROLL  0
#define PITCH 1
#define YAW   2
// Debug
#define DEBUG_LAT 0
#define DEBUG_LONG 0

#define R_EA 6378137.0 					// Ref 1 - Eq. 2.2
#define ECCENTRICITY 0.08181919   		// Ref 1 - Eq. 2.5

using namespace std;
using namespace arma;

// Classes
class guidance{
private:

	//////*************  Members *****************//////
	// Universal Iterator
	int i;
	int do_jump_count;
	float dt;

	// Takeoff variables
	bool take_off = true;
	bool wp_missed = false;
	bool no_more_arcs = false;

	// Path related variables - see ref 2
	float X_q,e_py,Q;
	colvec::fixed<3> q;
	colvec::fixed<3> qi;
	colvec::fixed<3> Qi;
	colvec::fixed<3> r;
	colvec::fixed<3> w;
	colvec::fixed<3> s;
	colvec::fixed<3> NEDp;
	colvec::fixed<3> e_path; 	// error relative to the desired path
	colvec::fixed<3> e_WP; 	// positional error (NED)
	colvec::fixed<3> r_temp;
	colvec::fixed<3> z1;
	colvec::fixed<3> z2;
	colvec::fixed<3> cf; // center of fillet for line arc line mechanism

	// Error
	float e_airspeed;

	// WP miss detection
	double min_wp_dist = 0;

	// Line - Arc - Line parameters
	double h;
	//int state = 1;
	int flag;
	int count = 1;
	float rf; // radius of fillet
	float t1;
	float t2;
	float H1;
	float H2;
	//float rho;
	int lambda; // carries the values +1 or -1 - determines the direction of the desired path
	float cd;
	float phi_co;
	//////*************  Methods *****************//////
	void path_manager();
	vec projection_error(vec u, vec v);
	void course_heading();
	void position_error();
//	void statemachine();

public:

	/* Members */
	// COMMENTED OUT PID FOR CALLING MRAC
	//pid Altitude;
	//pid heading2bank;
	//pid airspeed;
	//pid takeoff_heading;
	GPMRAC Altitude;
	GPMRAC heading2bank;
	GPMRAC airspeed;
	GPMRAC takeoff_heading;
	
	

	/* Methods */
	//Constructor function for the C++ class
	guidance();

	// Navigation and guidance
	void init(float);
	void update_attitude_cmd();
	void reset_to_runway();



	//Function destuctor
	virtual ~guidance();
};

#endif


