#ifndef PID_H
#define PID_H

#include <iostream>
#include <cmath>
#include <stack>
#include <ctime>

class PID
{
public:
    PID();
    void init(float kp, float ki, float kd,
              float limit_in, float tau_in,
              float Ts_in);

    float sat(float in, float limit);
    float controller_output(float y_c, float y, bool flag);
    float controller_output(float error, bool flag);

private:
    float K_p, K_d, K_i;
    float e_p, e_d, e_i, e_last;

    float Ts, limit, tau;

};

#endif // PID_H
