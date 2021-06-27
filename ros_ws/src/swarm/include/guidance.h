#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

#define M_PI 3.14159265358979323846 //pi

using namespace Eigen;

class guidance
{
public:
    guidance();
    void straightlineFollowing(Vector3d r, Vector3d q, Vector3d p, double course, double &alt_c, double& course_c);
    void orbitFollowing(Vector3d c, double rho, double lambda, Vector3d p, double course, double &alt_c, double& course_c);

private:
    double X_inf, Kpath, Korbit;
};

#endif // GUIDANCE_H
