#include <QCoreApplication>
#include "commlink.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    CommLink comm;

    return a.exec();
}
