#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

#define M_PI 3.14159265358979323846  /* pi */
#define R_EA 6378137.0
#define ECCENTRICITY 0.08181919

using namespace geometry_msgs;
using namespace std;

struct Home{
    Vector3 lla;
    double R_NE[3][3];
    Vector3 P_ecef;
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
    void convertLLA2ECEF(Vector3 lla, Vector3& ecef);
    void convertLLA2NED(Vector3 lla, Vector3& ned);

    // Home location
    void sethome(Vector3& lla);

    // Waypoint properties
    void appendWaypoints(int start_index, vector<MissionWP> wplist);
    void clearWaypoints(int start_index);

    // variables
    Home home;

private:

    vector<MissionWP> waypointslist;
    int curr_seq;

    //void readPlan(const Vector3& msg);

    // Node definition
    ros::NodeHandlePtr nav_node;

    // subscribe to planning data
    ros::Subscriber planning_sub;

    /// \brief Declare a ros publisher
    // ros::Publisher comm_writer_pub;

};

#endif // NAVIGATION_H
