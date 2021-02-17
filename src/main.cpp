#include <iostream>
#include <navigation.h>
#include <guidance.h>

#include <geometry_msgs/Vector3.h>

using namespace geometry_msgs;
using namespace std;

int main()
{

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



    /*

    // Navigation tests

    navigation nav;

    Vector3 lla, home_loc;
    Vector3 ecef;
    Vector3 ned;

    lla.x = 37.89124;
    lla.y = -120.98445;
    lla.z = 548;

    home_loc.x = 37.88129;
    home_loc.y = -120.99828;
    home_loc.z = 500;


    nav.sethome(home_loc);

    nav.convertLLA2NED(lla, ned);
    cout << "X = " <<ned.x << endl;
    cout << "Y = " <<ned.y << endl;
    cout << "Z = " <<ned.z << endl;

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
