#ifndef GNC_H
#define GNC_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>

#include <navigation.h>
#include <guidance.h>
#include <pid.h>

#include <ros/ros.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/HomePosition.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace Eigen;

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace mavros_msgs;

class GNC
{
public:
    GNC();

    // Function call to run the GNC loop
    void Run();

private:
    // Subscriber functions
    void getGPSData(const NavSatFix& msg);
    void getNEDData(const PoseStamped& msg);
    void getVelocityData(const TwistStamped& msg);
    void getHomeData(const HomePosition& msg);

    // Service functions
    bool getWaypointData(mavros_msgs::WaypointPush::Request &req,
                         mavros_msgs::WaypointPush::Response &res);

    // Node definition
    ros::NodeHandlePtr nav_node;

    // subscribe to planning data
    // ros::Subscriber planning_sub;
    ros::Subscriber gpsdata_sub;
    ros::Subscriber veldata_sub;
    ros::Subscriber neddata_sub;
    ros::Subscriber homedata_sub;

    // service for waypoint data
    ros::ServiceServer waypoint_service;

    // ros publisher for sending att data
    ros::Publisher att_control_pub;


    Vector3d lla, local_vel, local_pos;

    navigation nav;
    guidance guide;

    AttitudeTarget att_msg;

    bool HOMESET, WPSET;

};

#endif // GNC_H
