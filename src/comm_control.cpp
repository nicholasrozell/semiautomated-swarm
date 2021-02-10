#include "comm_control.h"

int
main(int argc, char **argv)
{
        // This program uses throw, wrap one big try/catch here
        try
        {
        int result = top(argc,argv);
                return result;
        }

        catch ( int error )
        {
                fprintf(stderr,"mavlink_control threw exception %i \n" , error);
                return error;
        }

}

int top(int argc, char **argv)
{

    // Default input arguments
#ifdef __APPLE__
    char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
    char *uart_name = (char*)"/dev/ttyUSB0";
#endif
    int baudrate = 57600;

    bool use_udp = true;
    char *udp_ip = (char*)"127.0.0.1";
    int udp_port = 14552; //14540;
    bool autotakeoff = true;

    // do the parse, will throw an int if it fails
    // parse_commandline(argc, argv, uart_name, baudrate, use_udp, udp_ip, udp_port, autotakeoff);

    Generic_Port *port;
    if(use_udp)
    {
        port = new UDP_Port(udp_ip, udp_port);
    }
    else
    {
        port = new Serial_Port(uart_name, baudrate);
    }

    //port_quit         = port;
    //autopilot_interface_quit = &autopilot_interface;
    //signal(SIGINT,quit_handler);

    comm_interface api(port);

    port->start();
    api.update();

    port->stop();
    delete port;

    return 0;
}
