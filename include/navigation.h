#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <thread>

#include <ros/ros.h>
//#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

#define M_PI 3.14159265358979323846  /* pi */
#define R_EA 6378137.0
#define ECCENTRICITY 0.08181919

//using namespace geometry_msgs;
using namespace Eigen;
using namespace std;

struct Home{
    Vector3d lla;
    double R_NE[3][3];
    Vector3d P_ecef;
};

struct MissionWP{
    float lat;
    float lon;
    float alt;
};

class navigation{

public:
    navigation();

    // Frame conversions
    void convertLLA2ECEF(Vector3d lla, Vector3d& ecef);
    void convertLLA2NED(Vector3d lla, Vector3d& ned);

    // Home location
    void sethome(Vector3d& lla);
    Vector3d gethome();

    // Waypoint properties
    void appendWaypoints(int start_index, vector<MissionWP> wplist);
    void clearWaypoints(int start_index);
    void updateWaypointCount();
    Vector3d getCurrWaypoint();
    int getCurrentSeq();

    vector<MissionWP> getcurrentWaypoints();


private:
    // variables
    Home home;

    vector<MissionWP> waypointslist;
    int curr_seq;

};

#endif // NAVIGATION_H
