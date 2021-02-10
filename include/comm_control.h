#ifndef COMMCONTROL_H
#define COMMCONTROL_H

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

using std::string;
using namespace std;

#include <common/mavlink.h>
#include <time.h>
#include <sys/time.h>

#include "comm_interface.h"
#include "serial_port.h"
#include "udp_port.h"

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int main(int argc, char **argv);
int top(int argc, char **argv);

// void commands(autopilot &autopilot_interface, bool autotakeoff);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate,
                bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff);

// quit handler
//autopilot *autopilot_interface_quit;
//Generic_Port *port_quit;
//void quit_handler( int sig );



#endif // COMMCONTROL_H
