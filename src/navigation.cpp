#include "navigation.h"

navigation::navigation()
{
    curr_seq = 2;

    // presets
    home.lla(0) = 0;
    home.lla(1) = 0;
    home.lla(2) = 0;

}

void navigation::convertLLA2ECEF(Vector3d lla, Vector3d& ecef)
{
    double lat, lon, alt;
    double b_by_a, a_by_b;

    b_by_a = SEMI_MINOR / SEMI_MAJOR; // value of b/a
    a_by_b = SEMI_MAJOR / SEMI_MINOR; // value of a/b

    // convert lat and long to radians
    lat = lla(0) * (M_PI / 180);
    lon = lla(1) * (M_PI / 180);
    alt = lla(2);

    ecef(0) = (SEMI_MAJOR/(sqrt(pow(cos(lat), 2) + pow(b_by_a, 2) * pow(sin(lat), 2))) + alt) * cos(lat) * cos(lon);
    ecef(1) = (SEMI_MAJOR/(sqrt(pow(cos(lat), 2) + pow(b_by_a, 2) * pow(sin(lat), 2))) + alt) * cos(lat) * sin(lon);
    ecef(2) = (SEMI_MINOR/(sqrt(pow(a_by_b, 2) * pow(cos(lat), 2) + pow(sin(lat), 2))) + alt) * sin(lat);

}

void navigation::convertLLA2NED(Vector3d lla, Vector3d& ned)
{

    Vector3d ecef, ecef_diff;
    Vector3d N, E, D;
    Vector3d N_0(0,0,1);

    Matrix3d R_N0, R_negE1;

    convertLLA2ECEF(lla, ecef);
    ecef_diff = ecef - home.P_ecef;

    R_N0 = AngleAxisd(lla(1)*M_PI/180, Vector3d(0,0,1));
    E = R_N0*Vector3d(0,1,0);

    R_negE1 = AngleAxisd(lla(0)*M_PI/180, -1*E);
    N = R_negE1*Vector3d(0,0,1);

    D = N.cross(E);

    ned(0) = ecef_diff.dot(N);
    ned(1) = ecef_diff.dot(E);
    ned(2) = ecef_diff.dot(D);

}

void navigation::sethome(Vector3d lla)
{

    Home home_loc;
    home_loc.lla = lla;

    convertLLA2ECEF(lla, home_loc.P_ecef);

    home = home_loc;

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

bool navigation::HalfplaneCheck(Vector3d position, Vector3d r, Vector3d n)
{
    Vector3d error;
    error = position - r;
    return error.dot(n) >=0;

}
