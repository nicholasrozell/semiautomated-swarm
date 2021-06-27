#ifndef COMMLINK_H
#define COMMLINK_H

#include <QObject>
#include <QtSerialPort/QtSerialPort>
#include <QUdpSocket>
#include <QThread>

#include "mavlink/v2.0/common/mavlink.h"
#include <ros/ros.h>
#include <mavros_msgs/WaypointList.h>
#include <geometry_msgs/TwistStamped.h>

using namespace mavros_msgs;
using namespace geometry_msgs;

class CommLink: public QObject
{
    Q_OBJECT
public:
    CommLink(QObject *parent = nullptr);
    virtual ~CommLink() {};


public slots:
    void Run();
    void RunSerial();

private:
    QUdpSocket *socket;
    QSerialPort *serial;

    mavlink_message_t global_mav;

    uint8_t systemID;
    bool BUSY, NEWNAVMSG;
    void ForwardQUDP();

    // Node definition
    ros::NodeHandlePtr serial_node;

    // subscribe to planning data
    // ros::Subscriber planning_sub;
    ros::Subscriber WPData_sub;

    // Subscriber functions
    void getWPData(const TwistStamped& msg);



};

#endif // COMMLINK_H
