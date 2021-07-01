#ifndef COMMLINK_H
#define COMMLINK_H

#include <iostream>
#include <QObject>
#include <QtSerialPort/QtSerialPort>
#include <QThread>

#include "mavlink/v2.0/common/mavlink.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/VehicleInfoGet.h>
#include <mavros_msgs/StreamRate.h>

using namespace std_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace mavros_msgs;

class CommLink: public QObject
{
    Q_OBJECT
public:
    CommLink(QObject *parent = nullptr);
    void Run();

//public slots:
//    void RunSerial();

private slots:
    void readData();

private:
    QSerialPort *serial;

    // Globals sys_status
    uint16_t voltage_battery; /*< [mV] Battery voltage*/
    int16_t current_battery; /*< [cA] Battery current, -1: autopilot does not measure the current*/
    int8_t battery_remaining;

    // Globals for VFR
    float airspeed; /*< [m/s] Current airspeed*/
    float groundspeed; /*< [m/s] Current ground speed*/
    float alt_vfr; /*< [m] Current altitude (MSL)*/
    float climb; /*< [m/s] Current climb rate*/
    int16_t heading; /*< [deg] Current heading in degrees, in compass units (0..360, 0=north)*/
    uint16_t throttle; /*< [%] Current throttle setting in integer percent, 0 to 100*/

    //Globals for Attitude
    float roll; /*< [rad] Roll angle (-pi..+pi)*/
    float pitch; /*< [rad] Pitch angle (-pi..+pi)*/
    float yaw; /*< [rad] Yaw angle (-pi..+pi)*/
    float rollspeed; /*< [rad/s] Roll angular speed*/
    float pitchspeed; /*< [rad/s] Pitch angular speed*/
    float yawspeed;

    //Globals global_position_int
    int32_t lat; /*< [degE7] Latitude, expressed*/
    int32_t lon; /*< [degE7] Longitude, expressed*/
    int32_t alt; /*< [mm] Altitude (AMSL). Note that virtually all GPS modules provide both WGS84 and AMSL.*/
    int32_t relative_alt; /*< [mm] Altitude above ground*/
    int16_t vx; /*< [cm/s] Ground X Speed (Latitude, positive north)*/
    int16_t vy; /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
    int16_t vz; /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
    uint16_t hdg;


    // Globals for nav controller output
    float nav_roll; /*< [deg] Current desired roll*/
    float nav_pitch; /*< [deg] Current desired pitch*/
    float alt_error; /*< [m] Current altitude error*/
    float aspd_error; /*< [m/s] Current airspeed error*/
    float xtrack_error; /*< [m] Current crosstrack error on x-y plane*/
    int16_t nav_bearing; /*< [deg] Current desired heading*/
    int16_t target_bearing; /*< [deg] Bearing to current waypoint/target*/
    uint16_t wp_dist;


    // Node definition
    ros::NodeHandlePtr comm_node;

    // Subscriber functions
    void BatteryState_Cb(const BatteryState& msg);
    void VFRHUD_Cb(const VFR_HUD& msg);
    void Orientation_Cb(const Imu& msg);
    void GlobalPos_Cb(const NavSatFix& msg);
    void GPSVel_Cb(const TwistStamped& msg);
    void RelativeAlt_Cb(const Float64& msg);
    void WPInfo_Cb(const Waypoint& msg);


    // subscribe to planning data
    // ros::Subscriber planning_sub;
    ros::Subscriber battery_sub;
    ros::Subscriber vfrhud_sub;
    ros::Subscriber orientation_sub;
    ros::Subscriber globalpos_sub;
    ros::Subscriber globalvel_sub;
    ros::Subscriber relalt_sub;
    ros::Subscriber Wpinfo_sub;


    // ros publisher for sending att data
    //ros::Publisher att_control_pub;
    //ros::Publisher Waypoint_pub;



};

#endif // COMMLINK_H
