#include "guidance.h"

guidance::guidance()
{
    // Line following params
    X_inf = 0.01;
    Kpath = 0.01;

    // Orbit params
    Korbit = 0.01;

}

void guidance::straightlineFollowing(Vector3d r, Vector3d q, Vector3d p, double course, double &alt_c, double& course_c)
{
    double Xq, epy;
    Vector3d ei_p, n, si;
    Vector3d ki(0, 0, -1);

    Xq = atan2(q(1), q(0));

    // position error in inertial frame
    ei_p = p - r;

    // n is vector normal to q-ki plane
    n = q.cross(ki);
    n = n/n.norm();

    // s is projection of e (e is error in path frame). si is s in inertial frame
    si = ei_p - (ei_p.dot(n))*n;

    alt_c = -r(2) + sqrt(pow(si(0), 2) + pow(si(1), 2)) * ( q(2)/sqrt(pow(q(0), 2) + pow(q(1), 2)) );

    while ((Xq - course) < -M_PI)
    {
        Xq += 2*M_PI;

    }

    while ((Xq - course) > M_PI)
    {
        Xq -= 2*M_PI;
    }

    epy = -sin(Xq*ei_p(0)) + cos(Xq*ei_p(1));

    course_c = Xq - X_inf*(2/M_PI)*atan(Kpath * epy);

}

void guidance::orbitFollowing(Vector3d c, double rho, double lambda,
                              Vector3d p, double course, double &alt_c,
                              double& course_c)
{

    double d, phi;

    alt_c = -1*c(2);
    d = sqrt(pow(p(0) - c(0), 2) + pow(p(1) - c(1), 2));

    phi = atan2(p(1) - c(1), p(0)- c(0));

    while ((phi - course) < -M_PI)
    {
        phi += 2*M_PI;

    }

    while ((phi - course) > M_PI)
    {
        phi -= 2*M_PI;
    }

    course_c = phi + lambda * (M_PI/2 + atan( Korbit * (d-rho)/rho ));


}
