#include "navigation.h"

navigation::navigation()
{
    curr_seq = 1;

    // presets
    home.lla(0) = 0;
    home.lla(1) = 0;
    home.lla(2) = 0;

}

void navigation::convertLLA2ECEF(Vector3d lla, Vector3d& ecef)
{
    Vector3d templla;
    double N_E;

    //Convert the lat and long to radians
    templla(0) = lla(0) * (M_PI / 180);
    templla(1) = lla(1) * (M_PI / 180);

    N_E = R_EA / sqrt(1 - pow(ECCENTRICITY, 2) * pow(sin(templla(0)), 2));

    // Transform to ECEF see eq 2.22 of ref 1
    ecef(0) = (N_E + templla(2)) * cos(templla(0)) * cos(templla(1));
    ecef(1) = (N_E + templla(2)) * cos(templla(0)) * sin(templla(1));
    ecef(2) = (N_E * (1 - pow(ECCENTRICITY, 2)) + templla(2))
            * sin(templla(0));

}

void navigation::convertLLA2NED(Vector3d lla, Vector3d& ned)
{

    Vector3d ecef;
    double a,b,c;
    convertLLA2ECEF(lla, ecef);

    a = home.R_NE[0][0] * (ecef(0) - home.P_ecef(0))
            + home.R_NE[0][1] * (ecef(1) - home.P_ecef(1))
            + home.R_NE[0][2] * (ecef(2) - home.P_ecef(2));

    b = home.R_NE[1][0] * (ecef(0) - home.P_ecef(0))
            + home.R_NE[1][1] * (ecef(1) - home.P_ecef(1))
            + home.R_NE[1][2] * (ecef(2) - home.P_ecef(2));

    c = home.R_NE[2][0] * (ecef(0) - home.P_ecef(0))
                + home.R_NE[2][1] * (ecef(1) - home.P_ecef(1))
                + home.R_NE[2][2] * (ecef(2) - home.P_ecef(2));

    ned(0) = a;
    ned(1) = b;
    ned(2) = -(double) (lla(2) - c);

}

void navigation::sethome(Vector3d& lla)
{

    double N_E;
    Home temp_home;

    //Convert the lat and long to radians
    temp_home.lla(0) = lla(0) * (M_PI / 180);
    temp_home.lla(1) = lla(1) * (M_PI / 180);
    temp_home.lla(2) = lla(2);

    // First row
    temp_home.R_NE[0][0] = -sin(temp_home.lla(0)) * cos(temp_home.lla(1));
    temp_home.R_NE[0][1] = -sin(temp_home.lla(0)) * sin(temp_home.lla(1));
    temp_home.R_NE[0][2] = cos(temp_home.lla(0));

    // Second row
    temp_home.R_NE[1][0] = -sin(temp_home.lla(1));
    temp_home.R_NE[1][1] = cos(temp_home.lla(1));
    temp_home.R_NE[1][2] = 0;

    // Third row
    temp_home.R_NE[2][0] = -cos(temp_home.lla(0)) * cos(temp_home.lla(1));
    temp_home.R_NE[2][1] = -cos(temp_home.lla(0)) * sin(temp_home.lla(1));
    temp_home.R_NE[2][2] = -sin(temp_home.lla(0));


    N_E = R_EA / sqrt(1 - pow(ECCENTRICITY, 2) * pow(sin(temp_home.lla(0)), 2));

    temp_home.P_ecef(0) = (N_E + temp_home.lla(2)) * cos(temp_home.lla(0))
            * cos(temp_home.lla(1));
    temp_home.P_ecef(1) = (N_E + temp_home.lla(2)) * cos(temp_home.lla(0))
            * sin(temp_home.lla(1));
    temp_home.P_ecef(2) = (N_E * (1 - pow(ECCENTRICITY, 2)) + temp_home.lla(2))
            * sin(temp_home.lla(0));


    // Lat and Long should be in degrees
    temp_home.lla(0) *= 180/M_PI;
    temp_home.lla(1) *= 180/M_PI;

    home = temp_home;

}

void navigation::appendWaypoints(int start_index, vector<MissionWP> wplist)
{
    waypointslist.erase(waypointslist.begin()+start_index, waypointslist.end());
    waypointslist.insert(waypointslist.begin()+start_index, wplist.begin(), wplist.end());
}

void navigation::clearWaypoints(int start_index)
{
    waypointslist.erase(waypointslist.begin()+start_index, waypointslist.end());
}

void navigation::updateWaypointCount()
{
    // only update if curr_seq is less than size of mission
    if(curr_seq < (int)waypointslist.size()-1){
        curr_seq++;
    }
}

Vector3d navigation::gethome()
{
    return home.lla;
}

vector<MissionWP> navigation::getcurrentWaypoints()
{
    return waypointslist;
}

int navigation::getCurrentSeq()
{
    return curr_seq;
}

Vector3d navigation::getCurrWaypoint()
{
    Vector3d vec_wp;

    MissionWP wp;

    wp = waypointslist.at(curr_seq);
    vec_wp(0) = wp.lat;
    vec_wp(1) = wp.lon;
    vec_wp(2) = wp.alt;

    return vec_wp;
}
