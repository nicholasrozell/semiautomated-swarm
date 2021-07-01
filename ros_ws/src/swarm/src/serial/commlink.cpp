#include "commlink.h"

CommLink::CommLink(QObject *parent) :
    QObject(parent)
{

    ROS_INFO("Initializing Serial Link");

    // Create a node name for communicating with pixhawk
    std::string nodename = "SerialNode";

    // Create the ros node
    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodename);
    }
    this->comm_node.reset(new ros::NodeHandle(nodename));

    int system_id;
    ros::param::get("/mavros/target_system_id", system_id);

    // create a topic name to read messages and create a subscriber
    std::string batterydata_topic = "/mavros/battery";
    this->battery_sub = this->comm_node->subscribe(batterydata_topic, 10,
                                                   &CommLink::BatteryState_Cb, this);
    std::string vfrhud_topic = "/mavros/vfr_hud";
    this->battery_sub = this->comm_node->subscribe(vfrhud_topic, 10,
                                                   &CommLink::VFRHUD_Cb, this);

    std::string orientation_topic = "/mavros/imu/data";
    this->orientation_sub = this->comm_node->subscribe(orientation_topic, 10,
                                                      &CommLink::Orientation_Cb, this);

    std::string globalpos_topic = "/mavros/global_position/global";
    this->globalpos_sub = this->comm_node->subscribe(globalpos_topic, 10,
                                                     &CommLink::GlobalPos_Cb, this);

    std::string globalvel_topic = "/mavros/global_position/raw/gps_vel";
    this->globalvel_sub = this->comm_node->subscribe(globalvel_topic, 10,
                                                     &CommLink::GPSVel_Cb, this);

    std::string relalt_topic = "/mavros/global_position/rel_alt";
    this->relalt_sub = this->comm_node->subscribe(relalt_topic, 10,
                                                  &CommLink::RelativeAlt_Cb, this);

    std::string WpInfo_topic = "/control/path";
    this->Wpinfo_sub = this->comm_node->subscribe(WpInfo_topic, 10,
                                                  &CommLink::WPInfo_Cb, this);

    alt_error = 0;
    aspd_error= 0;
    xtrack_error = 0;

    nav_roll = -433;
    nav_pitch = 1231;
    nav_bearing = -998;

    wp_dist = 1;
    target_bearing = -10;

    ROS_INFO("Started all subscribers");
    // create a QSerialport
    serial = new QSerialPort();
    serial->setPortName("/dev/ttyUSB0");

    if(!serial->setBaudRate(QSerialPort::Baud115200))
        qDebug() << serial->errorString();
    if(!serial->setDataBits(QSerialPort::Data8))
        qDebug() << serial->errorString();
    if(!serial->setParity(QSerialPort::NoParity))
        qDebug() << serial->errorString();
    if(!serial->setFlowControl(QSerialPort::HardwareControl))
        qDebug() << serial->errorString();
    if(!serial->setStopBits(QSerialPort::OneStop))
        qDebug() << serial->errorString();
    if(!serial->open(QIODevice::ReadWrite))
        qDebug() << serial->errorString();


    ROS_INFO("Initialized serial connection");


    //connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
    Run();


}

void CommLink::BatteryState_Cb(const BatteryState& msg)
{
    battery_remaining = msg.percentage*100;
    current_battery = msg.current*100;
    voltage_battery = msg.voltage*1000;

}

void CommLink::VFRHUD_Cb(const VFR_HUD& msg)
{

    airspeed = msg.airspeed;
    groundspeed = msg.groundspeed;
    heading = msg.heading;
    throttle = msg.throttle*100;
    climb = msg.climb;
    alt_vfr = msg.altitude;

}

void CommLink::Orientation_Cb(const Imu& msg)
{
    tf::Quaternion rot_quat, q_new;
    rot_quat.setRPY( M_PI, 0, M_PI/2 );

    tf::Quaternion q(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w);

    q_new = rot_quat*q;
    q_new.normalize();

    tf::Matrix3x3 m(q_new);
    double roll_get, pitch_get, yaw_get;
    m.getRPY(roll_get, pitch_get, yaw_get);

    roll = roll_get;
    pitch = pitch_get;
    yaw = yaw_get;

    roll += 3.14;
    //std::cout << "Roll = " << roll << std::endl;
    //std::cout << "Pitch = " << pitch << std::endl;
    //std::cout << "Yaw = " << yaw << std::endl;

    rollspeed = msg.angular_velocity.x;
    pitchspeed = msg.angular_velocity.y;
    yawspeed = msg.angular_velocity.z;

}

void CommLink::GlobalPos_Cb(const NavSatFix& msg)
{

    lat = msg.latitude*1e7;
    lon = msg.longitude*1e7;
    alt = msg.altitude*1e3;

}

void CommLink::GPSVel_Cb(const TwistStamped& msg)
{
    vx = msg.twist.linear.x*1e2;
    vy = msg.twist.linear.y*1e2;
    vz = msg.twist.linear.z*1e2;
}

void CommLink::RelativeAlt_Cb(const Float64& msg)
{
    relative_alt = msg.data*1e3;


}

void CommLink::WPInfo_Cb(const Waypoint& msg)
{

    alt_error = msg.x_lat;
    aspd_error= msg.y_long;
    xtrack_error = msg.z_alt;

    nav_roll = msg.param2;
    nav_pitch = msg.param3;
    nav_bearing = msg.param4;

    wp_dist = msg.param1;
    target_bearing = msg.command;


}

void CommLink::Run()
{


    ros::ServiceClient stream_client = this->comm_node->serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    stream_client.waitForExistence();

    mavros_msgs::StreamRate stream;
    stream.request.stream_id = 0;
    stream.request.message_rate = 10;
    stream.request.on_off = 1;

    stream_client.call(stream);

    ROS_INFO("Stream Rate Set Maybe?!?");

    mavros_msgs::VehicleInfoGet vehicle_info;
    ros::ServiceClient client = this->comm_node->serviceClient<mavros_msgs::VehicleInfoGet>("/mavros/vehicle_info_get");
    client.waitForExistence();

    uint8_t buffermsg[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    int cBuffer;
    QByteArray bytes;

    ros::Time last_heartbeat_time(0), last_sys_status_time(0), last_vfr_hud_time(0);
    ros::Time last_att_time(0), last_global_pos_time(0), last_nav_control_time(0);

    while (ros::ok()){
            // Get the current time
            ros::Time time_now = ros::Time::now();

            if((time_now-last_heartbeat_time) > ros::Duration(1.0))
            {
                vehicle_info.request.sysid = 0;
                vehicle_info.request.compid = 0;
                vehicle_info.request.get_all = 1;

                if (client.call(vehicle_info))
                {

                    //Create heartbeat message
                    mavlink_heartbeat_t heartbeat;
                    heartbeat.custom_mode = vehicle_info.response.vehicles[0].custom_mode;
                    heartbeat.type = vehicle_info.response.vehicles[0].type;
                    heartbeat.autopilot = vehicle_info.response.vehicles[0].autopilot;
                    heartbeat.base_mode = vehicle_info.response.vehicles[0].base_mode;
                    heartbeat.system_status = vehicle_info.response.vehicles[0].system_status;

                    // Encode message
                    mavlink_msg_heartbeat_encode(1, 1, &msg, &heartbeat);

                    // Send heartbeat message
                    memset(buffermsg, 0, MAVLINK_MAX_PACKET_LEN);
                    cBuffer = mavlink_msg_to_send_buffer(buffermsg, &msg);
                    //bytes((char *)buffermsg, cBuffer);
                    bytes.append((char *)buffermsg, cBuffer);

                    serial->write(bytes);
                    serial->flush();


                    bytes.clear();
                    last_heartbeat_time = time_now;
                }
            }

            if((time_now-last_sys_status_time) > ros::Duration(1.0))
            {

                //Create sys status message
                mavlink_sys_status_t sys_status;
                sys_status.onboard_control_sensors_present = 3;//onboard_control_sensors_present;
                sys_status.onboard_control_sensors_enabled = 3;//onboard_control_sensors_enabled;
                sys_status.onboard_control_sensors_health = 3;//onboard_control_sensors_health;
                sys_status.load = 0;//load;
                sys_status.voltage_battery = voltage_battery;
                sys_status.current_battery = current_battery;
                sys_status.drop_rate_comm = 0;//drop_rate_comm;
                sys_status.errors_comm = 0;//errors_comm;
                sys_status.errors_count1 = 0;//errors_count1;
                sys_status.errors_count2 = 0;//errors_count2;
                sys_status.errors_count3 = 0;//errors_count3;
                sys_status.errors_count4 = 0;//errors_count4;
                sys_status.battery_remaining = battery_remaining;
                // Encode message
                mavlink_msg_sys_status_encode(1,1, &msg, &sys_status);

                // Send sys_status message
                memset(buffermsg, 0, MAVLINK_MAX_PACKET_LEN);
                cBuffer = mavlink_msg_to_send_buffer(buffermsg, &msg);
                //bytes((char *)buffermsg, cBuffer);
                bytes.append((char *)buffermsg, cBuffer);

                serial->write(bytes);
                serial->flush();

                bytes.clear();
                last_sys_status_time = time_now;

            }

            if((time_now-last_vfr_hud_time) > ros::Duration(0.1))
            {
                //Create VFR HUD message
                mavlink_vfr_hud_t vfr_hud;
                vfr_hud.airspeed = airspeed;
                vfr_hud.groundspeed = groundspeed;
                vfr_hud.alt = alt;
                vfr_hud.climb = climb;
                vfr_hud.heading = heading;
                vfr_hud.throttle = throttle;
                // Encode message
                mavlink_msg_vfr_hud_encode(1,1, &msg, &vfr_hud);

                // Send VFR HUD message
                memset(buffermsg, 0, MAVLINK_MAX_PACKET_LEN);
                cBuffer = mavlink_msg_to_send_buffer(buffermsg, &msg);
                //bytes((char *)buffermsg, cBuffer);
                bytes.append((char *)buffermsg, cBuffer);

                serial->write(bytes);
                serial->flush();

                bytes.clear();
                last_vfr_hud_time = time_now;
            }

            if((time_now-last_att_time) > ros::Duration(0.2))
            {
                // Create Attitude message
                mavlink_attitude_t attitude;
                attitude.time_boot_ms = 0;//time_boot_ms;
                attitude.roll = roll;
                attitude.pitch = pitch;
                attitude.yaw = yaw;
                attitude.rollspeed = rollspeed;
                attitude.pitchspeed = pitchspeed;
                attitude.yawspeed = yawspeed;

                // Encode message
                mavlink_msg_attitude_encode(1,1, &msg, &attitude);

                // Send Attitude message
                memset(buffermsg, 0, MAVLINK_MAX_PACKET_LEN);
                cBuffer = mavlink_msg_to_send_buffer(buffermsg, &msg);
                //bytes((char *)buffermsg, cBuffer);
                bytes.append((char *)buffermsg, cBuffer);

                serial->write(bytes);
                serial->flush();

                bytes.clear();
                last_att_time = time_now;
            }

            if((time_now - last_global_pos_time) > ros::Duration(0.2))
            {
                // Create global position int message
                mavlink_global_position_int_t global_pos_int;
                global_pos_int.time_boot_ms = 0;//time_boot_ms;
                global_pos_int.lat = lat;
                global_pos_int.lon = lon;
                global_pos_int.alt = alt;
                global_pos_int.relative_alt = relative_alt;
                global_pos_int.vx = vx;
                global_pos_int.vy = vy;
                global_pos_int.vz = vz;
                global_pos_int.hdg = heading*1e2;

                // Encode Global Position Int message
                mavlink_msg_global_position_int_encode(1,1,&msg,&global_pos_int);

                // Send Global Position Int message
                memset(buffermsg, 0, MAVLINK_MAX_PACKET_LEN);
                cBuffer = mavlink_msg_to_send_buffer(buffermsg, &msg);
                //bytes((char *)buffermsg, cBuffer);
                bytes.append((char *)buffermsg, cBuffer);

                serial->write(bytes);
                serial->flush();

                bytes.clear();
                last_global_pos_time = time_now;
            }

            if((time_now - last_nav_control_time) > ros::Duration(0.5))
            {

                // Create Nav controller output message as spoof for Waypoint path message

                // * This message is used to send current waypoint and current position information to the GCS
                // * Since mission planner doesnt have customizing capabilities we use redefine variables in this messsage
                // * to act as Current WP and Current Position
                //*
                // * 1.nav_roll -> CurrentWP X location(North)
                // * 2.nav_pitch -> CurrentWP Y location(East)
                // * 3.nav_bearing -> CurrentWp Z location(Down)
                // * 4.wp_dist -> Current Wp sequence
                // * 5.target_bearing -> Total Waypoints in sequence
                // * 6.alt_error -> Current position X
                // * 7.aspd_error -> Current position Y
                // * 8.xtrack_error -> Current position Z


                mavlink_nav_controller_output_t pathinfo;
                pathinfo.nav_roll = nav_roll;
                pathinfo.nav_pitch = nav_pitch;
                pathinfo.alt_error = alt_error;
                pathinfo.aspd_error = aspd_error*100;
                pathinfo.xtrack_error = xtrack_error;
                pathinfo.nav_bearing = -1*nav_bearing;
                pathinfo.target_bearing = target_bearing;
                pathinfo.wp_dist = wp_dist;

                // Encode nav controller message
                mavlink_msg_nav_controller_output_encode(1,1,&msg,&pathinfo);

                // Send nav controller message
                memset(buffermsg, 0, MAVLINK_MAX_PACKET_LEN);
                cBuffer = mavlink_msg_to_send_buffer(buffermsg, &msg);
                //bytes((char *)buffermsg, cBuffer);
                bytes.append((char *)buffermsg, cBuffer);

                serial->write(bytes);
                serial->flush();

                bytes.clear();
                last_nav_control_time = time_now;
            }

            //ros::Duration(0.01).sleep();
            ros::spinOnce();
    }


}

void CommLink::readData()
{
    QByteArray data = serial->readAll();

}
