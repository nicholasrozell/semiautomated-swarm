#include "commlink.h"

CommLink::CommLink(QObject *parent) :
    QObject(parent)
{

    // create a QUDP socket
    socket = new QUdpSocket(this);
    socket->bind(QHostAddress::LocalHost, 14551);

    // create a QSerialport
    serial = new QSerialPort();
    //serial->setPortName("COM4");
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


    connect(socket, SIGNAL(readyRead()), this, SLOT(Run()));
    //connect(serial, SIGNAL(readyRead()), this, SLOT(RunSerial()));

}


void CommLink::ForwardQUDP()
{

    uint8_t buffermsg[MAVLINK_MAX_PACKET_LEN];
    QByteArray buffer;

    buffer.resize(socket->pendingDatagramSize());

    QHostAddress sender;
    quint16 senderPort;
    qint64 len;

    len = socket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

    if (len > 0)
    {
        // Something received - print out all bytes and parse packet
        mavlink_message_t msg;
        mavlink_status_t status;

        for (int i = 0; i < len; ++i)
        {

            // After reading data on socket/serial always parse the data into the message and send
            // If not it will send a additional junk on the channel which can slow down the communication
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
            {
                qDebug() << "Sys: " << msg.sysid << " Len: " << msg.len << "Msg id: " << msg.msgid;
                memset(buffermsg, 0, MAVLINK_MAX_PACKET_LEN);

                int cBuffer = mavlink_msg_to_send_buffer(buffermsg, &msg);
                QByteArray bytes((char *)buffermsg, cBuffer);

                // Identify the right message and forward the message
                // Only sending sending six messages for now to reduce load
                switch (msg.msgid)
                {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    serial->write(bytes);
                    serial->flush();
                    break;

                }
                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    serial->write(bytes);
                    serial->flush();
                    break;

                }
                case MAVLINK_MSG_ID_POWER_STATUS:
                {
                    serial->write(bytes);
                    serial->flush();
                    break;
                }
                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    serial->write(bytes);
                    serial->flush();
                    break;
                }
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    serial->write(bytes);
                    serial->flush();
                    break;
                }
                case MAVLINK_MSG_ID_VFR_HUD:
                {
                    serial->write(bytes);
                    serial->flush();
                    break;
                }

                } //end switch

            } //end if

        } // end for

    } // end if len>0

    /*
     * READING SERIAL CODE WAS HERE
     */

    QThread::usleep(50);

}

void CommLink::RunSerial()
{

        qint64 len_recv;
        QByteArray datas = serial->readAll();
        len_recv = datas.length();

        mavlink_message_t msg_recv;
        mavlink_status_t status_recv;

        if (len_recv >0){

            // Data length is non zero
            for (int i = 0; i < len_recv; ++i)
            {
                // Need to parse on a different channel from MAVLINK_COMM_0 as thats used for sending.
                if (mavlink_parse_char(MAVLINK_COMM_1, datas[i], &msg_recv, &status_recv))
                {
                     switch (msg_recv.msgid) {
                     case MAVLINK_MSG_ID_MISSION_ITEM:
                     {
                         mavlink_mission_item_t mav_mission;
                         mavlink_msg_mission_item_decode(&msg_recv, &mav_mission);

                         qDebug() << "Lat = "<<mav_mission.x;
                         qDebug() << "Long = "<<mav_mission.y;
                         qDebug() << "Alt = "<<mav_mission.z;
                         qDebug() << "Mission type " << mav_mission.mission_type;

                         qDebug() << "===============================";


                     }
                     case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                     {
                         mavlink_request_data_stream_t data_stream;
                         mavlink_msg_request_data_stream_decode(&msg_recv, &data_stream);

                         qDebug() << "stream id: " << data_stream.req_stream_id;

                     }

                     }

                }

            }

        }

}

void CommLink::Run()
{
   ForwardQUDP();
}
