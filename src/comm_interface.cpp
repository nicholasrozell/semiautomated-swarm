#include "comm_interface.h"

comm_interface::comm_interface(Generic_Port *port_)
{

    reading_status = 0;      // whether the read thread is running
    writing_status = 0;      // whether the write thread is running

    port = port_; // port management object

    // Create a node name for communicating with pixhawk
    std::string nodename = "commNode";

    // Create the ros node
    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodename);
    }

    // create a topic name to read messages to comm
    std::string topicName = "/pixhawk/data";

    ROS_INFO("comm node ready");
    this->comm_node.reset(new ros::NodeHandle(nodename));

    // create a publisher to publish comm messages
    this->comm_writer_pub = this->comm_node->advertise<mavros_msgs::Mavlink>("/comm/data", 1000);

    // subscribe to the topic to get messages to pixhawk
    this->comm_reader_sub = this->comm_node->subscribe(topicName, 1000, &comm_interface::write_mavlink_msgs, this);

    // set writing status to true as messages need to be written to pixhawk
    writing_status = true;

}

void comm_interface::update()
{

    while(ros::ok())
    {
        //this->read_mavlink_msgs();
        ros::spinOnce();
    }

}

/*
 * Read the mavlink messages on the ros topic
 * and write it to the comm device
 */
void comm_interface::write_mavlink_msgs(const mavros_msgs::Mavlink& msg)
{
    mavlink_message_t toCommMsg;

    std::cout << "reading on mavros " << msg.msgid <<std::endl;

    // Convert mavros type to mavlink_message_t type
    this->convert(msg, toCommMsg);

    // Send the message
    int len = port->write_message(toCommMsg);
}

/*
 * Read the messages from comm device on
 * the serial/UDP and write to the ros topic
 */
void comm_interface::read_mavlink_msgs()
{
    bool success;               // receive success flag
    bool received_all = false;  // receive only one message


    // Blocking wait for new data
    while ( !received_all )
    {
    // ----------------------------------------------------------------------
    //   READ MESSAGE
    // ----------------------------------------------------------------------
    mavlink_message_t message;

    mavros_msgs::Mavlink fromCommMsg;
    success = port->read_message(message);

    if(success)
    {
        // If you get a complete message convert it from mavlink_message_t
        this->convert(message, fromCommMsg);
        //std::cout << message.msgid << ", "<< fromCommMsg.msgid << std::endl;
        this->comm_writer_pub.publish(fromCommMsg);

    }

    // give the write thread time to use the port
    if ( writing_status > false ) {
            usleep(10000); // look for data of batches at 100Hz
   }

    } // end: while not received all

}

/*
 * Convert mavros msg to mavlink_message_t
 */

inline bool comm_interface::convert(const mavros_msgs::Mavlink &rmsg, mavlink_message_t &mmsg)
{

    if (rmsg.payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0]))
    {
        return false;
    }

     mmsg.magic = MAVLINK_STX;
     mmsg.len = rmsg.len;
     mmsg.seq = rmsg.seq;
     mmsg.sysid = rmsg.sysid;
     mmsg.compid = rmsg.compid;
     mmsg.msgid = rmsg.msgid;
     mmsg.checksum = rmsg.checksum;
     std::copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64);

     return true;

}

/*
 * Convert mavlink_message_t to mavros msg
 */

inline bool comm_interface::convert(const mavlink_message_t &mmsg, mavros_msgs::Mavlink &rmsg)
{
    const size_t payload64_len = (mmsg.len + 7) / 8;

     rmsg.len = mmsg.len;
     rmsg.seq = mmsg.seq;
     rmsg.sysid = mmsg.sysid;
     rmsg.compid = mmsg.compid;
     rmsg.msgid = mmsg.msgid;
     rmsg.checksum = mmsg.checksum;
     rmsg.payload64 = std::move(mavros_msgs::Mavlink::_payload64_type(mmsg.payload64, mmsg.payload64 + payload64_len));

     return true;

}
