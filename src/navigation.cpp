#include "navigation.h"

navigation::navigation()
{
    // Create a node name for communicating with pixhawk
    std::string nodename = "navNode";

    // Create the ros node
    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodename);
    }

    // create a topic name to read messages to comm
    std::string topicName = "/swarm/planning";

    ROS_INFO("Navigation Node Ready");
    this->nav_node.reset(new ros::NodeHandle(nodename));

    // create a publisher to publish comm messages
    // this->planning_sub = this->nav_node->subscribe(topicName, 1000, &navigation::readPlan, this);
}

void navigation::convertLLA2ECEF(Vector3 lla, Vector3& ecef)
{
    Vector3 templla;
    double N_E;

    //Convert the lat and long to radians
    templla.x = lla.x * (M_PI / 180);
    templla.y = lla.y * (M_PI / 180);

    N_E = R_EA / sqrt(1 - pow(ECCENTRICITY, 2) * pow(sin(templla.x), 2));

    // Transform to ECEF see eq 2.22 of ref 1
    ecef.x = (N_E + templla.z) * cos(templla.x) * cos(templla.y);
    ecef.y = (N_E + templla.z) * cos(templla.x) * sin(templla.y);
    ecef.z = (N_E * (1 - pow(ECCENTRICITY, 2)) + templla.z)
            * sin(templla.x);

}

void navigation::convertLLA2NED(Vector3 lla, Vector3& ned)
{

    Vector3 ecef;
    double a,b,c;
    convertLLA2ECEF(lla, ecef);

    a = home.R_NE[0][0] * (ecef.x - home.P_ecef.x)
            + home.R_NE[0][1] * (ecef.y - home.P_ecef.y)
            + home.R_NE[0][2] * (ecef.z - home.P_ecef.z);

    b = home.R_NE[1][0] * (ecef.x - home.P_ecef.x)
            + home.R_NE[1][1] * (ecef.y - home.P_ecef.y)
            + home.R_NE[1][2] * (ecef.z - home.P_ecef.z);

    c = home.R_NE[2][0] * (ecef.x - home.P_ecef.x)
                + home.R_NE[2][1] * (ecef.y - home.P_ecef.y)
                + home.R_NE[2][2] * (ecef.z - home.P_ecef.z);

    ned.x = a;
    ned.y = b;
    ned.z = -(double) (lla.z - c);


}

void navigation::sethome(Vector3& lla)
{

    double N_E;
    Home temp_home;

    //Convert the lat and long to radians
    temp_home.lla.x = lla.x * (M_PI / 180);
    temp_home.lla.y = lla.y * (M_PI / 180);
    temp_home.lla.z = lla.z;

    // First row
    temp_home.R_NE[0][0] = -sin(temp_home.lla.x) * cos(temp_home.lla.y);
    temp_home.R_NE[0][1] = -sin(temp_home.lla.x) * sin(temp_home.lla.y);
    temp_home.R_NE[0][2] = cos(temp_home.lla.x);

    // Second row
    temp_home.R_NE[1][0] = -sin(temp_home.lla.y);
    temp_home.R_NE[1][1] = cos(temp_home.lla.y);
    temp_home.R_NE[1][2] = 0;

    // Third row
    temp_home.R_NE[2][0] = -cos(temp_home.lla.x) * cos(temp_home.lla.y);
    temp_home.R_NE[2][1] = -cos(temp_home.lla.x) * sin(temp_home.lla.y);
    temp_home.R_NE[2][2] = -sin(temp_home.lla.x);


    N_E = R_EA / sqrt(1 - pow(ECCENTRICITY, 2) * pow(sin(temp_home.lla.x), 2));

    temp_home.P_ecef.x = (N_E + temp_home.lla.z) * cos(temp_home.lla.x)
            * cos(temp_home.lla.y);
    temp_home.P_ecef.y = (N_E + temp_home.lla.z) * cos(temp_home.lla.x)
            * sin(temp_home.lla.y);
    temp_home.P_ecef.z = (N_E * (1 - pow(ECCENTRICITY, 2)) + temp_home.lla.z)
            * sin(temp_home.lla.x);

    home = temp_home;

}

void navigation::appendWaypoints(int start_index, vector<MissionWP> wplist)
{
    waypointslist.erase(waypointslist.begin()+start_index, waypointslist.end());
    waypointslist.insert(waypointslist.begin()+start_index, wplist.begin(), wplist.end());
}

void navigation::clearWaypoints(int start_index)
{
    waypointslist.erase(waypointslist.begin()+start_index, waypointslist.end());
}
