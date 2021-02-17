#include "../global/global.h"
#include <iostream>

// Namespaces
using namespace std;

// constructor of the structure navigation
navigation::navigation(void) {

}

// destructor of the structure navigation
navigation::~navigation(void) {
	WPList.erase(WPList.begin(), WPList.end());
}

// function that, when provided with all set of parameters, adds a new waypoint to the flight plan
void navigation::addPoint(short s_frame, short s_command, short s_current,
		short s_autocontinue, float s_param1, float s_param2, float s_param3,
		float s_param4, float s_x, float s_y, float s_z) {

	waypoint temp;
	temp.param1 = s_param1;
	temp.param2 = s_param2;
	temp.param3 = s_param3;
	temp.param4 = s_param4;

	// values that will be kept as original regardless
	// of the frame type
	temp.x = s_x;
	temp.y = s_y;
	temp.z = s_z;

	temp.frame = s_frame;
	temp.command = s_command;
	temp.current = s_current;
	temp.autocontinue = s_autocontinue;

	if (temp.frame == 0) {

		temp.lat = s_x;
		temp.lon = s_y;
		temp.h = s_z;

		//Convert the lat and long to radians
		temp.lat *= (M_PI / 180);
		temp.lon *= (M_PI / 180);

		N_E = R_EA / sqrt(1 - pow(ECCENTRICITY, 2) * pow(sin(temp.lat), 2));

		// Transform to ECEF see eq 2.22 of ref 1
		temp.P_ecef[0] = (N_E + temp.h) * cos(temp.lat) * cos(temp.lon);
		temp.P_ecef[1] = (N_E + temp.h) * cos(temp.lat) * sin(temp.lon);
		temp.P_ecef[2] = (N_E * (1 - pow(ECCENTRICITY, 2)) + temp.h)
				* sin(temp.lat);

		// zero out the P_n vector
		P_n[0] = 0;
		P_n[1] = 0;
		P_n[2] = 0;

		// Equation 2-23
		for (int i = 0; i <= 2; i++) {
			for (int j = 0; j <= 2; j++) {
				P_n[i] += home.R_NE[i][j] * (temp.P_ecef[j] - home.P_ecef[j]);
			}
		}

		temp.n = P_n[0];
		temp.e = P_n[1];
		temp.d = (double) -temp.h;
				//P_n[2];

		//Convert the lat and long back to deg
		temp.lat *= (180 / M_PI);
		temp.lon *= (180 / M_PI);

		// Debug
//		cout << "temp.lat " << temp.lat << " temp.lon " << temp.lon << " temp.h " << temp.h << endl;
//		printf("\n N_E %.5f", N_E);
//		printf("\n P_ecef[0] %.5f  P_ecef[1] %.5f  P_ecef[2] %.5f", temp.P_ecef[0], temp.P_ecef[1], temp.P_ecef[2]);
//		printf("\n P_n[0] %.5f  P_n[1] %.5f  P_n[2] %.5f", P_n[0], P_n[1], P_n[2]);

	} else if (temp.frame == 1) {
		// Provisions to go from NED to GPS
		temp.n = s_x;
		temp.e = s_y;
		temp.d = s_z;

	}

	if(temp.command == 16) {
		temp.airspeed_cmd = temp.param1;
		temp.tolerance = temp.param2;
		//cout << "tol " << temp.tolerance << " airspeed " << temp.airspeed_cmd << endl;
	}

	WPList.push_back(temp);

}

// function that prints out a flight plan
// first row -- virtual variables
// second row -- actual direct variables for the flight plan
void navigation::print(void) {

	printf(
			"frame command current autocontinue  param1  param2  param3  param4   x     y    z     lat    long     h    n    e    d \n");
	//Print out the various waypoints
	for (int i = 0; i < WPList.size(); i++) {
		printf("%d %d %d %d ", WPList[i].frame, WPList[i].command,
				WPList[i].current, WPList[i].autocontinue);
		printf("%.5f %.5f %.5f %.5f %.5f %.5f %.1f ", WPList[i].param1,
				WPList[i].param2, WPList[i].param3, WPList[i].param4,
				WPList[i].x, WPList[i].y, WPList[i].z);
		printf("%.5f %.5f %.5f %.5f %.5f %.5f ", WPList[i].lat, WPList[i].lon,
				WPList[i].h, WPList[i].n, WPList[i].e, WPList[i].d);
		printf("\n");
	}
}

// returns current size of the flight plan
int navigation::count(void) {
	return WPList.size();
}

void navigation::changehome(float lat_in, float lon_in, float h_in) {

	double N_E;
	home_location temp_home;
//	cout << "lat " << lat_in << " lon " << lon_in << " h " << h_in << endl;
	//Convert the lat and long to radians
	temp_home.lat = lat_in * (M_PI / 180);
	temp_home.lon = lon_in * (M_PI / 180);
	temp_home.h = h_in;

	// First row
	temp_home.R_NE[0][0] = -sin(temp_home.lat) * cos(temp_home.lon);
	temp_home.R_NE[0][1] = -sin(temp_home.lat) * sin(temp_home.lon);
	temp_home.R_NE[0][2] = cos(temp_home.lat);

	// Second row
	temp_home.R_NE[1][0] = -sin(temp_home.lon);
	temp_home.R_NE[1][1] = cos(temp_home.lon);
	temp_home.R_NE[1][2] = 0;

	// Third row
	temp_home.R_NE[2][0] = -cos(temp_home.lat) * cos(temp_home.lon);
	temp_home.R_NE[2][1] = -cos(temp_home.lat) * sin(temp_home.lon);
	temp_home.R_NE[2][2] = -sin(temp_home.lat);

//	cout << "R_NE " << endl;
//	cout << temp_home.R_NE[0][0] << "\t" << temp_home.R_NE[0][1] << "\t"
//			<< temp_home.R_NE[0][2] << endl;
//	cout << temp_home.R_NE[1][0] << "\t" << temp_home.R_NE[1][1] << "\t"
//			<< temp_home.R_NE[1][2] << endl;
//	cout << temp_home.R_NE[2][0] << "\t" << temp_home.R_NE[2][1] << "\t"
//			<< temp_home.R_NE[2][2] << endl;

	N_E = R_EA / sqrt(1 - pow(ECCENTRICITY, 2) * pow(sin(temp_home.lat), 2));

//	cout << "N_E " << N_E << endl;

	temp_home.P_ecef[0] = (N_E + temp_home.h) * cos(temp_home.lat)
			* cos(temp_home.lon);
	temp_home.P_ecef[1] = (N_E + temp_home.h) * cos(temp_home.lat)
			* sin(temp_home.lon);
	temp_home.P_ecef[2] = (N_E * (1 - pow(ECCENTRICITY, 2)) + temp_home.h)
			* sin(temp_home.lat);

//	cout << "P_ecef " << endl;
//	cout << temp_home.P_ecef[0] << "\t" << temp_home.P_ecef[1] << "\t"
//			<< temp_home.P_ecef[2] << endl;

	home = temp_home;

}

// For debugging only
//	printf("\nR_NE[0][0]  %f R_NE[0][1]  %f R_NE[0][2]  %f", R_NE[0][0] , R_NE[0][1], R_NE[0][2]);
//	printf("\nR_NE[1][0]  %f R_NE[1][1]  %f R_NE[1][2]  %f", R_NE[1][0] , R_NE[1][1], R_NE[1][2]);
//	printf("\nR_NE[2][0]  %f R_NE[2][1]  %f R_NE[2][2]  %f", R_NE[2][0] , R_NE[2][1], R_NE[2][2]);
//	printf("\nN_E %f",N_E);
//	printf("\nP_ecef_ref[0]  %f P_ecef_ref[1]  %f P_ecef_ref[2]  %f", P_ecef_ref[0] , P_ecef_ref[1], P_ecef_ref[2]);
