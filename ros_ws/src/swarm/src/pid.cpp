#include "pid.h"

PID::PID()
{

}

void PID::init(float kp, float ki, float kd, float limit_in, float tau_in,
                float Ts_in) {

        // Assigns gains locally
        K_p = kp;
        K_i = ki;
        K_d = kd;

        e_p = 0;
        e_i = 0;
        e_d = 0;
        e_last = 0;

        Ts = Ts_in;
        limit = limit_in;
        tau = tau_in;

}

float PID::sat(float in, float limit) {

        // Check to see if the
        if (abs(in) > limit) {
                return copysign(limit, in);
        }

        return in;
}

float PID::controller_output(float y_c, float y, bool flag) {

    float u, u_unsat;
        // If the pid has been flagged persistent variables
        if (flag == true) {
                e_i = 0;
                e_d = 0;
                e_last = 0;
        }

        // Calculate errors
        e_p = y_c - y;						// Proportional error
        e_i = e_i + Ts / 2 * (e_p + e_last);	// integrator error
        e_d = (2 * tau - Ts) / (2 * tau + Ts) * e_d
                        + (2 / (2 * tau + Ts)) * (e_p - e_last);	// Derivative error
        e_last = e_p;			// Store the current value for the next iteration

        // Calculate the raw output of the pid
        u_unsat = K_p * e_p + K_i * e_i + K_d * e_d;
        // Check for saturation
        u = sat(u_unsat, limit);

        //Check to see if anti-windup measures are needed
        if (K_i != 0) {
                e_i = e_i + Ts / K_i * (u - u_unsat);
        }

        return u;

}

float PID::controller_output(float error, bool flag) {

    float u, u_unsat;

        // If the pid has been flagged persistent variables
        if (flag == 1) {
            e_i = 0;
            e_d = 0;
            e_last = 0;
        }

        // Calculate errors
        e_p = error;						// Proportional error
        e_i = e_i + Ts / 2 * (e_p + e_last);	//integrator error
        e_d = (2 * tau - Ts) / (2 * tau + Ts) * e_d
                        + (2 / (2 * tau + Ts)) * (e_p - e_last);	// Derivative error
        e_last = e_p;			// Store the current value for the next iteration

       // Calculate the raw output of the pid
       u_unsat = K_p * e_p + K_i * e_i + K_d * e_d;

        // Check for saturation
        u = sat(u_unsat, limit);

        //Check to see if anti-windup measures are needed
        if (K_i != 0) {
                e_i = e_i + Ts / K_i * (u - u_unsat);
        }

        return u;

}
