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
    std::string neddata_topic = "/mavros/local_position/pose";
    this->neddata_sub = this->nav_node->subscribe(neddata_topic, 1000, &GNC::getNEDData, this);

    std::string veldata_topic = "/mavros/local_position/velocity_local";
    this->veldata_sub = this->nav_node->subscribe(veldata_topic, 1000, &GNC::getVelocityData, this);

    std::string home_topic = "/mavros/home_position/home";
    this->homedata_sub = this->nav_node->subscribe(home_topic, 1000, &GNC::getHomeData, this);

    std::string planningdata_service = "/control/waypoints";
    this->waypoint_service = this->nav_node->advertiseService(planningdata_service, &GNC::getWaypointData, this);

    this->att_control_pub = this->nav_node->advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    this->Waypoint_pub = this->nav_node->advertise<mavros_msgs::WaypointList>("/control/path", 10);

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

void GNC::getNEDData(const PoseStamped& msg)
{
    // Local position is given in ENU frame
    // Convert to position in NED for guidance and navigation
    local_pos(0) = msg.pose.position.y;
    local_pos(1) = msg.pose.position.x;
    local_pos(2) = -1*msg.pose.position.z;


}

void GNC::getVelocityData(const TwistStamped& msg)
{
    // Local velocity is returned in ENU frame
    // Convert to NED for passing to guidance and navigation functions

    local_vel(0) = msg.twist.linear.y;
    local_vel(1) = msg.twist.linear.x;
    local_vel(2) = msg.twist.linear.z;
}

void GNC::PublishWpPath()
{
    WaypointList wp_msg;

    wp_msg.current_seq = nav.getCurrentSeq();

    vector<MissionWP> path = nav.getcurrentWaypoints();

    std::cout << "pathsize = " << path.size() << std::endl;
    for (std::vector<MissionWP>::iterator it = path.begin(); it != path.end(); ++it)
    {
        //std::cout << path.at(i).lat << std::endl;

        Waypoint wp;
        wp.x_lat = (*it).lat;
        wp.y_long = (*it).lon;
        wp.z_alt = (*it).alt;


        wp_msg.waypoints.push_back(wp);
    }
    this->Waypoint_pub.publish(wp_msg);
}

bool GNC::getWaypointData(WaypointPush::Request &req,
                          WaypointPush::Response &res)
{

    std::cout << "start_index = "<< req.start_index << std::endl;
    std::cout << "Total Wp got = "<< req.waypoints.size() << std::endl;

    vector<MissionWP> wplist;
    MissionWP wp_temp;
    Vector3d home;

    home = nav.gethome();

    for (int i=0; i< req.waypoints.size(); i++){

        wp_temp.lat = req.waypoints[i].x_lat;
        wp_temp.lon = req.waypoints[i].y_long;
        wp_temp.alt = req.waypoints[i].z_alt;

        if (req.waypoints[i].frame == 0){

            wp_temp.alt = req.waypoints[i].z_alt - home(2);
        }

        wplist.push_back( wp_temp);

    }
    res.success = true;
    res.wp_transfered = req.waypoints.size();

    nav.appendWaypoints(req.start_index, wplist);
    if(!WPSET){ 
        WPSET = true;
    }
    return true;
}

void GNC::getHomeData(const HomePosition &msg)
{
    Vector3d home, home_prev;

    home(0) = msg.geo.latitude;
    home(1) = msg.geo.longitude;
    home(2) = msg.geo.altitude;

    home_prev = nav.gethome();

    if (home(0)!= home_prev(0) ||
        home(1)!= home_prev(1) ||
        home(2)!= home_prev(2))
    {
        // if different from current home, then set
        nav.sethome(home);
        ROS_INFO("HOME position set/updated");

    }
    else
    {
        ROS_INFO("HOME not updated...remove this message later after testing home changed");
    }

    HOMESET = true;

}

void GNC::InitPID()
{
    dt = .025; //~40 Hz based on input

    float K_COURSE_P(2), K_COURSE_I(0), K_COURSE_D(0.00001);
    float K_ALT_P(0.075), K_ALT_I(0.0), K_ALT_D(0.00001);
    float K_VEL_P(0.1), K_VEL_I(0.0001), K_VEL_D(0.2);
    float LIMIT_ROLL(45*M_PI/180), LIMIT_PITCH(30*M_PI/180), LIMIT_THRUST(1.0);

    course_pid.init(K_COURSE_P, K_COURSE_I, K_COURSE_D, LIMIT_ROLL, 0, dt);
    alt_pid.init(K_ALT_P, K_ALT_I, K_ALT_D, LIMIT_PITCH, 0, dt);
    vel_pid.init(K_VEL_P, K_VEL_I, K_VEL_D, LIMIT_THRUST, 0, dt);

}

void GNC::Run()
{
    double course;
    double course_c, alt_c;
    float pitch, roll, yaw(90*M_PI/180); // yaw is fixed since not used

    double rho(5), lambda(1);


    Vector3d lla_temp, vel_temp, ned_currpos, ned_wp, ned_wp_prev;
    Vector3d curr_wp, prev_wp, home, r, q, c;
    Quaternionf quat;

    InitPID();

    // Wait until home is set
    while (!HOMESET)
    {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    // Once home is set, get the current value
    home = nav.gethome();

    // Set the first waypoint to home location
    prev_wp = home;
    prev_wp(2) += 50;

    // Wait till good GPS values are set
    while (lla.norm() <= 0)
    {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    while (ros::ok()){

        if (lla.norm() > 0) // only if messages got
        {
            // Get current LLA, Velocity and Course
            lla_temp = lla;
            vel_temp = local_vel;
            ned_currpos = local_pos;

            course = atan2(local_vel(1), local_vel(0));
            if(course < 0)
            {
                course += 2*M_PI;
            }

            PublishWpPath();

            if (WPSET && nav.getCurrentSeq() >= 0)
            {
                curr_wp = nav.getCurrWaypoint();
                //curr_wp(2) += home(2);

                // convert WP to NED
                nav.convertLLA2NED(curr_wp, ned_wp);
                nav.convertLLA2NED(prev_wp, ned_wp_prev);

                ROS_INFO_STREAM("currpos "<<ned_currpos(0) <<", " << ned_currpos(1) << ", "<< ned_currpos(2));
                ROS_INFO_STREAM("NED Wp "<<ned_wp(0) <<", " << ned_wp(1) << ", "<< ned_wp(2));
                ROS_INFO_STREAM("NED WpPrev "<<ned_wp_prev(0) <<", " << ned_wp_prev(1) << ", "<< ned_wp_prev(2));
                ROS_INFO_STREAM("NED q "<<q(0) <<", " << q(1) << ", "<< q(2));

                vector<MissionWP> WP = nav.getcurrentWaypoints();
                std::cout << nav.getCurrentSeq() << ", " << WP.size() << std::endl;
                bool motion_type = nav.getCurrentSeq() < WP.size()-1;
                //std::cout << motion_type << std::endl;


                if (motion_type){
                    //std::cout << "Executing straightline manoeuvre" << std::endl;
                    r = ned_wp_prev;
                    q = ned_wp - ned_wp_prev;
                    q = q/q.norm();

                    guide.straightlineFollowing(r, q, ned_currpos, course, alt_c, course_c);

                }
                else
                {
                    //std::cout << "Executing orbit manoeuvre" << std::endl;
                    c = ned_wp;
                    guide.orbitFollowing(c, rho, lambda, ned_currpos, course, alt_c, course_c);

                }

                if(nav.HalfplaneCheck(ned_currpos, ned_wp, q)){
                   nav.updateWaypointCount();
                   prev_wp = curr_wp;
                }

                pitch = alt_pid.controller_output(alt_c, -1*ned_currpos(2), false);
                //std::cout << "pitch = " << pitch << ", alt_c = " <<alt_c << ", curr height = " << -ned_currpos(2) << std::endl;

                roll = course_pid.controller_output(course_c, course, false);
                //std::cout << "roll = " << roll << ", course_c = " <<course_c << ", curr course = " << course << std::endl;

                quat = AngleAxisf(pitch, Vector3f::UnitX())
                    * AngleAxisf(roll, Vector3f::UnitY())
                    * AngleAxisf(yaw, Vector3f::UnitZ()); // roll is y axis and pitch is x axis in mavros body frame

                att_msg.type_mask = 0b00000100;
                att_msg.orientation.w = quat.w();
                att_msg.orientation.x = quat.x();
                att_msg.orientation.y = quat.y();
                att_msg.orientation.z = quat.z();

                att_msg.thrust = 0.8;

                att_msg.body_rate.x = 0;
                att_msg.body_rate.y = 0;
                att_msg.body_rate.z = 0;

                this->att_control_pub.publish(att_msg);

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
