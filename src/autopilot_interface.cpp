#include "autopilot_interface.h"

autopilot_interface::autopilot_interface(Generic_Port *port_)
{

    reading_status = 0;      // whether the read thread is running
    writing_status = 0;      // whether the write thread is running

    port = port_; // port management object

    // Create a node name for communicating with pixhawk
    std::string nodename = "pixhawkNode";

    // Create the ros node
    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodename);
    }

    // create a topic name to read messages to pixhawk
    std::string topicName = "/comm/data";

    ROS_INFO("pixhawk node ready");
    this->pixhawk_node.reset(new ros::NodeHandle(nodename));

    // create a publisher to publish pixhawk messages
    this->pixhawk_writer_pub = this->pixhawk_node->advertise<mavros_msgs::Mavlink>("/pixhawk/data", 10);

    // subscribe to the topic to get messages to pixhawk
    this->pixhawk_reader_sub = this->pixhawk_node->subscribe(topicName, 5, &autopilot_interface::read_mavlink_msgs, this);

    // set writing status to true as messages need to be written to pixhawk
    writing_status = true;

}

void autopilot_interface::update()
{

    while(ros::ok())
    {
        this->write_mavlink_msgs();
        ros::spinOnce();

    }

}

/*
 * Read the mavlink messages on the ros topic
 * and write it on serial/UDP to the pixhawk
 */
void autopilot_interface::read_mavlink_msgs(const mavros_msgs::Mavlink& msg)
{
    mavlink_message_t toPixmsg;

    // Convert mavros type to mavlink_message_t type
    this->convert(msg, toPixmsg);

    // Send the message
    int len = port->write_message(toPixmsg);
}

/*
 * Read the messages from pixhawk on the serial/UDP
 * and write to the ros topic
 */
void autopilot_interface::write_mavlink_msgs()
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

    mavros_msgs::Mavlink fromPixmsg;
    success = port->read_message(message);

    if(success)
    {

        switch (message.msgid)
        {

            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                    printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_heartbeat_t heartbeat;

                    mavlink_msg_heartbeat_decode(&message, &heartbeat);

                    printf("basemode = %d and custom mode = %d\n", heartbeat.base_mode, heartbeat.custom_mode);
                    //break;
            }

            case MAVLINK_MSG_ID_SYSTEM_TIME:
            {
                printf("MAVLINK_MSG_ID_SYSTEM_TIME\n");
                mavlink_system_time_t systemtime;

                mavlink_msg_system_time_decode(&message, &systemtime);
                std::cout << "time is "<< systemtime.time_boot_ms <<std::endl;


            }

            case MAVLINK_MSG_ID_SYS_STATUS:
            {
                   // printf("MAVLINK_MSG_ID_SYS_STATUS\n");

                    //break;
            }
        }


        // If you get a complete message convert it from mavlink_message_t
        this->convert(message, fromPixmsg, mavros_msgs::Mavlink::FRAMING_OK);
        // std::cout << message.msgid << ", "<< fromPixmsg.msgid << std::endl;
        this->pixhawk_writer_pub.publish(fromPixmsg);
        break;

    }

    // give the write thread time to use the port
    if ( writing_status > false ) {
            usleep(100); // look for components of batches at 10kHz
        }

    } // end: while not received all

}

/*
 * Convert mavros msg to mavlink_message_t
 */

inline bool autopilot_interface::convert(const mavros_msgs::Mavlink &rmsg, mavlink_message_t &mmsg)
{


    if (rmsg.payload64.size() > sizeof(mmsg.payload64) / sizeof(mmsg.payload64[0])) {
            return false;
        }

    if (!rmsg.signature.empty() && rmsg.signature.size() != sizeof(mmsg.signature)) {
        return false;
    }


    mmsg.magic = rmsg.magic;
        mmsg.len = rmsg.len;
        mmsg.incompat_flags = rmsg.incompat_flags;
        mmsg.compat_flags = rmsg.compat_flags;
        mmsg.seq = rmsg.seq;
        mmsg.sysid = rmsg.sysid;
        mmsg.compid = rmsg.compid;
        mmsg.msgid = rmsg.msgid;
        mmsg.checksum = rmsg.checksum;
        // [[[end]]] (checksum: 2ef42a7798f261bfd367bf4157b11ec0)
        std::copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64);
        std::copy(rmsg.signature.begin(), rmsg.signature.end(), mmsg.signature);

     //mmsg.magic = MAVLINK_STX;
     //mmsg.len = rmsg.len;
     //mmsg.seq = rmsg.seq;
     //mmsg.sysid = rmsg.sysid;
     //mmsg.compid = rmsg.compid;
     //mmsg.msgid = rmsg.msgid;
     //mmsg.checksum = rmsg.checksum;
     //std::copy(rmsg.payload64.begin(), rmsg.payload64.end(), mmsg.payload64);

     return true;

}

/*
 * Convert mavlink_message_t to mavros msg
 */

inline bool autopilot_interface::convert(const mavlink_message_t &mmsg, mavros_msgs::Mavlink &rmsg, uint8_t framing_status)
{

    const size_t payload64_len = (mmsg.len + 7) / 8;

    rmsg.framing_status = framing_status;

    // [[[cog:
    // for f in FIELD_NAMES:
    //     cog.outl("rmsg.%s = mmsg.%s;" % (f, f))
    // ]]]

    rmsg.magic = mmsg.magic;
    rmsg.len = mmsg.len;
    rmsg.incompat_flags = mmsg.incompat_flags;
    rmsg.compat_flags = mmsg.compat_flags;
    rmsg.seq = mmsg.seq;
    rmsg.sysid = mmsg.sysid;
    rmsg.compid = mmsg.compid;
    rmsg.msgid = mmsg.msgid;
    rmsg.checksum = mmsg.checksum;

    rmsg.payload64 = mavros_msgs::Mavlink::_payload64_type(mmsg.payload64, mmsg.payload64 + payload64_len);
    //rmsg.payload64 = std::move(mavros_msgs::Mavlink::_payload64_type(mmsg.payload64, mmsg.payload64 + payload64_len));

     // copy signature block only if message is signed
     if (mmsg.incompat_flags & MAVLINK_IFLAG_SIGNED)
         rmsg.signature = mavros_msgs::Mavlink::_signature_type(mmsg.signature, mmsg.signature + sizeof(mmsg.signature));
     else
         rmsg.signature.clear();


     return true;

}
