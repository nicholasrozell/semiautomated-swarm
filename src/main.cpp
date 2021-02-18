#include <iostream>
#include <gnc.h>
#include <navigation.h>


using namespace std;

int main()
{

    GNC gnc;
    gnc.Run();

    /*
    float dt = 0;

    dt = .025; //~40 Hz based on input

    PID course_pid, alt_pid, vel_pid;

    float K_COURSE_P(0.1), K_COURSE_I(0.0001), K_COURSE_D(0.2);
    float K_ALT_P(0.1), K_ALT_I(0.0001), K_ALT_D(0.2);
    float K_VEL_P(0.1), K_VEL_I(0.0001), K_VEL_D(0.2);
    float LIMIT_ROLL(60*M_PI/180), LIMIT_PITCH(30*M_PI/180), LIMIT_THRUST(1.0);

    course_pid.init(K_COURSE_P, K_COURSE_I, K_COURSE_D, LIMIT_ROLL, 0, dt);
    alt_pid.init(K_ALT_P, K_ALT_I, K_ALT_D, LIMIT_PITCH, 0, dt);
    vel_pid.init(K_VEL_P, K_VEL_I, K_VEL_D, LIMIT_THRUST, 0, dt);

    guidance guide;

    Vector3d r(1,2,-5);
    Vector3d q(1,1,-1);
    Vector3d p(4,0,-5);
    double course, alt_c, course_c;

    course = 1.8;

    guide.straightlineFollowing(r,q, p, course, alt_c, course_c);

    cout << "Alt " << alt_c <<" coursec " << course_c <<std::endl;

    Vector3d c(1,2,-5);
    double rho(1.0), lambda(1) ;
    Vector3d p1(4,0,-5);
    guide.orbitFollowing(c,rho, lambda, p1, course, alt_c, course_c);

    cout << "Alt " << alt_c <<" coursec " << course_c <<std::endl;
    */


    /*

    // Navigation tests

    navigation nav;

    Vector3d lla, home_loc;
    Vector3d ecef;
    Vector3d ned;

    lla(0) = 37.89124;
    lla(1) = -120.98445;
    lla(2) = 548;

    home_loc(0) = lla(0);//37.88129;
    home_loc(1) = lla(1);//-120.99828;
    home_loc(2) = 500;


    nav.sethome(home_loc);

    nav.convertLLA2NED(lla, ned);
    cout << "X = " <<ned(0) << endl;
    cout << "Y = " <<ned(1) << endl;
    cout << "Z = " <<ned(2) << endl;

    vector<int> a, b;
    for (int i =0; i<10;i++)
    {
        a.push_back(i);
    }

    for(int j =0; j <=4; j++)
    {
        b.push_back(j);
    }

    cout << "size of a " << a.size() <<"\n";
    for(int i=0; i < a.size(); i++)
        cout << a.at(i) << ' ';

    a.erase(a.begin()+10,a.end());

    a.insert(a.begin()+10, b.begin(), b.end());
    cout << "\n";
    cout << "size of a " << a.size() <<"\n";

    for(int i=0; i < a.size(); i++)
        cout << a.at(i) << ' ';
    cout << "\n";

    */
    return 0;
}
