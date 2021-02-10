#ifndef COMM_INTERFACE_H
#define COMM_INTERFACE_H

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <common/mavlink.h>

#include "generic_port.h"


class comm_interface
{
public:
    comm_interface(Generic_Port *port_);
    void update();

private:

    char reading_status;
    char writing_status;

    /// \brief Create a ros node ptr
    ros::NodeHandlePtr comm_node;

    /// \brief Declare a ros subscriber
    ros::Subscriber comm_reader_sub;

    /// \brief Declare a ros publisher
    ros::Publisher comm_writer_pub;

    /// \brief Create a generic port to communicate on serial/UDP
    Generic_Port *port;

    void write_mavlink_msgs(const mavros_msgs::Mavlink& msg);
    void read_mavlink_msgs();
    bool convert(const mavros_msgs::Mavlink &rmsg, mavlink_message_t &mmsg);
    bool convert(const mavlink_message_t &mmsg, mavros_msgs::Mavlink &rmsg);
};

#endif // COMM_INTERFACE_H
