#include "gnc.h"

GNC::GNC()
{
    // Create a node name for communicating with pixhawk
    std::string nodename = "gncNode";

    // Create the ros node
    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodename);
    }

    ROS_INFO("GNC Node Ready");
    this->nav_node.reset(new ros::NodeHandle(nodename));

    // create a topic name to read messages and create a subscriber
    std::string gpsdata_topic = "/mavros/global_position/global";
    this->gpsdata_sub = this->nav_node->subscribe(gpsdata_topic, 1000, &GNC::getGPSData, this);

    // create a topic name to read messages and create a subscriber
    std::string neddata_topic = "/mavros/global_position/local";
    this->neddata_sub = this->nav_node->subscribe(neddata_topic, 1000, &GNC::getNEDData, this);


    std::string veldata_topic = "/mavros/local_position/velocity_local";
    this->veldata_sub = this->nav_node->subscribe(veldata_topic, 1000, &GNC::getVelocityData, this);

    std::string planningdata_topic = "/control/waypoints";
    this->planning_sub = this->nav_node->subscribe(planningdata_topic, 1000, &GNC::getWaypointData, this);


    // create a publisher to publish attitude messages
    this->att_control_pub = this->nav_node->advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    lla(0) = 0;
    lla(1) = 0;
    lla(2) = 0;

    local_vel(0) = 0;
    local_vel(1) = 0;
    local_vel(2) = 0;

    HOMESET = false;
    WPSET = false;

}

void GNC::getGPSData(const NavSatFix& msg)
{
    lla(0) = msg.latitude;
    lla(1) = msg.longitude;
    lla(2) = msg.altitude;


}

void GNC::getNEDData(const Odometry& msg)
{
    // Local position is given in ENU frame
    // Convert to position in NED for guidance and navigation
    local_pos(0) = msg.pose.pose.position.y;
    local_pos(1) = msg.pose.pose.position.x;
    local_pos(2) = -1*msg.pose.pose.position.z;


}

void GNC::getVelocityData(const TwistStamped& msg)
{
    // Local velocity is returned in ENU frame
    // Convert to NED for passing to guidance and navigation functions

    local_vel(0) = msg.twist.linear.y;
    local_vel(1) = msg.twist.linear.x;
    local_vel(2) = msg.twist.linear.z;
}

void GNC::getWaypointData(const WaypointList& msg)
{
    vector<MissionWP> wplist;
    MissionWP wp_temp;
    Vector3d home;

    home = nav.gethome();

    for (int i=0; i< msg.waypoints.size(); i++){

        wp_temp.lat = msg.waypoints[i].x_lat;
        wp_temp.lon = msg.waypoints[i].y_long;
        wp_temp.alt = msg.waypoints[i].z_alt;

        if (msg.waypoints[i].frame == 0){

            wp_temp.alt = msg.waypoints[i].z_alt - home(2);
        }

        // std::cout << i<< "," <<wp_temp.lat <<", "<<  msg.waypoints[i].x_lat << std::endl;

        wplist.push_back( wp_temp);

    }

    if(!WPSET){

        nav.appendWaypoints(msg.current_seq, wplist);
        WPSET = true;
    }

}

void GNC::Run()
{
    double course;
    Vector3d lla_temp, vel_temp, ned_temp, home_temp;

    float dt = 0;

    dt = .025; //~40 Hz based on input

    PID course_pid, alt_pid, vel_pid;
    Vector3d curr_wp;

    float K_COURSE_P(0.1), K_COURSE_I(0.0001), K_COURSE_D(0.2);
    float K_ALT_P(0.1), K_ALT_I(0.0001), K_ALT_D(0.2);
    float K_VEL_P(0.1), K_VEL_I(0.0001), K_VEL_D(0.2);
    float LIMIT_ROLL(60*M_PI/180), LIMIT_PITCH(30*M_PI/180), LIMIT_THRUST(1.0);

    course_pid.init(K_COURSE_P, K_COURSE_I, K_COURSE_D, LIMIT_ROLL, 0, dt);
    alt_pid.init(K_ALT_P, K_ALT_I, K_ALT_D, LIMIT_PITCH, 0, dt);
    vel_pid.init(K_VEL_P, K_VEL_I, K_VEL_D, LIMIT_THRUST, 0, dt);

    home_temp(0) = -35.363262;
    home_temp(1) = 149.1652372;
    home_temp(2) = 584.0900268;

    nav.sethome(home_temp);

    ROS_INFO("Home location set");


    while (ros::ok()){

        if (lla.norm() > 0) // only if messages got
        {
            // Get current LLA, Velocity and Course
            lla_temp = lla;
            vel_temp = local_vel;

            // nav.convertLLA2NED(lla_temp, ned_temp); // Use ros message instead of function in navigation clas
            ned_temp = local_pos;

            course = atan2(local_vel(1), local_vel(0));//*180/M_PI
            if(course < 0)
            {
                course += 2*M_PI;
            }

            if (WPSET && nav.getCurrentSeq() >= 0)
            {
                curr_wp = nav.getCurrWaypoint();

                ROS_INFO_STREAM(nav.getCurrentSeq() << "," <<curr_wp(0) <<", " << curr_wp(1) << ", "<< curr_wp(2));

                nav.updateWaypointCount();
                ros::Duration(0.1).sleep();
            }
            else{
                ROS_INFO("Waypoints not set");
                ros::Duration(1).sleep();
            }
        }
        ros::spinOnce();

    }

}
