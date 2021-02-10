#include "mavlink_control.h"

/*
void
commands(autopilot &api, bool autotakeoff)
{

    // arm the vehicle
    if(autotakeoff){
        api.arm_disarm(true);
    }

    usleep(1000);
    // set take off mode
    api.set_mode(217, 13);

    // sleep for 30 secs
    usleep(30000000);

    // set vehicle in guided
    api.set_mode(217, 15);

    int counter(0);
    printf("Sending attitude commands");
    while (counter < 300){
        printf("Counter = %d\n", counter );
        if (counter < 100){
            api.set_attitude_thrust(10,0,0,0.8);
        }else if(counter >=100 && counter < 200){
            api.set_attitude_thrust(-10,0,0,0.6);
        }
        else{
            api.set_attitude_thrust(10,-10,0,0.3);
        }

        counter++;

        usleep(500000);
    }


    //api.set_mode(89,4);
    //api.takeoff(20);

    //usleep(20000000);
    printf("Setting throttle to 0.6\n");
    //api.set_attitude(0.7);
    //api.set_mode(209, 0);
    //printf("set to manual mode");
    //api.setservo(3,1500); //set servo 3 to override throttle doesnt work. Need to use RC_CHANNELS_OVERRIDE
    usleep(15000000);
    //printf("Changing to guided mode");
    //api.set_mode(217, 15);
    //usleep(5000000);
    //api.set_mode(81,2); // setting mode works

    //printf("Takeoff to 10 m above ground");
    //api.takeoff(10);
    //usleep(10000000);


	return;

}
*/
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
    int udp_port = 14551; //14540;
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

    autopilot_interface api(port);

    port->start();
    api.update();

    port->stop();
    delete port;

    return 0;
}
