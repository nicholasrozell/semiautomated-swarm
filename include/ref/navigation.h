#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <stdio.h>
#include "../global/global.h"

#ifndef NAVPLAN_H_DEFINED
#define NAVPLAN_H_DEFINED

#define R_EA 6378137.0 					// Ref 1 - Eq. 2.2
#define ECCENTRICITY 0.08181919   		// Ref 1 - Eq. 2.5

// class describes one NAV point structure
// all values with underscores are actual parameters and WITHOUT underscores are functions that
// either allow to SET a value of the actual parameter or to read it.
// This is needed since the structure of the actual vanpoint needs to be retained, while allowing
// flexibility. QGC allows to use the same parameter as several values depending on the type of waypoint.
// In order not to create multiple instances of the same variable with different names a technique
// described below was used:
//
// READING:
//			blah = waypoint.e();     <---reading value of e into the blahblah

// SETTING:
//			waypoint.e(5.234);       <---changing value of e in the waypoint to 5.234
//			waypoint.lon(5.234);       <---changing value of lon in the waypoint to 5.234
//		(!) Keep in mind that these two values will be assigned to the same ACTUAL variable _x
// 			the same can be achieved by using direct assignment that is described next:

// USING ACTUAL VARIABLES
// 			blah = waypoint._x;   <--- reading value of _x into blah
//			waypoint._x = 5.234;  <--- setting a new value of _x

// Frame Type possible values:
// 0 - Global/Abs Alt
// 3 - Global/Rel Alt
// 1 - Local (NED)
// 2 - Mission

// Point type possible values
// 16 - NAV waypoint
// 22 - NAV take off
// 17 - NAV loiter unlim
// 19 - NAV loiter time
// 18 - NAV loiter turns
// 20 - NAV Ret. to Launch
// 21 - NAV land
// 112- IF Delay over
// 117- DO Jump to index
// 16 - Other

// Allows for the transformation of coordinate frames as well as adding waypoints dynamically
class navigation {
private:
	///// Members /////

	// Coordinate transformation variables - see ref 1
	double N_E;
	double P_ecef[3];
	double P_ecef_ref[3];
	double P_n[3];
	double R_NE[3][3];


public:
	///// Members /////
	// NA

	//// Methods ////
	// Constructor
	navigation(void);

	// adds a waypoint to a flight plan using
	// (frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z)
	void addPoint(short s_frame, short s_command, short s_current, short s_autocontinue, float s_param1, float s_param2, float s_param3, float s_param4,
			float s_x, float s_y, float s_z);
	void changehome(float lat_in, float lon_in, float h_in);
	// function that prints out a flight plan
	void print(void);

	// returns current size of flight plan ()
	int count(void);

	// Destructor
	~navigation(void);
};

#endif
