#ifndef COMMLINK_H
#define COMMLINK_H

#include <QObject>
#include <QtSerialPort/QtSerialPort>
#include <QUdpSocket>
#include <QThread>

#include "mavlink/v2.0/common/mavlink.h"

class CommLink: public QObject
{
    Q_OBJECT
public:
    CommLink(QObject *parent = nullptr);


public slots:
    void Run();
    void RunSerial();

private:
    QUdpSocket *socket;
    QSerialPort *serial;

    void ForwardQUDP();
};

#endif // COMMLINK_H
