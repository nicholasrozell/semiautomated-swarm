/*%%%%%%%%%%%%%%%%%%% Fixed Wing guidance  %%%%%%%%%%%%%%%%%%%%%%%%%%%
 % Author: Jacob Stockton
 % Date: 2/19/14
 % Purpose: OSU Stabilis Autopilot guidance and Guidance.
 %
 % Ref 1:  Unmanned Rotorcraft Systems. Guowei Cai, Ben M. Chen, Tong Heng Lee
 % Ref 2:  Small Unmanned Aircraft: Theory and Practice. Beard, Randall. Mclain, Timothy
 %
 %
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// Standard Library
#include <armadillo>
#include <cmath>

// Local includes
#include "../global/global.h"
//Namespaces
using namespace std;
using namespace arma;

/***************************** guidance Class *******************************/

#define WP_MISS_TOL 500
#define t1 -50
#define t2 -10
//////////////////  Constructor /////////////////////
guidance::guidance ()
{

}
////////////////////////////////////////////////////

/////////////////// Private ////////////////////////
void
guidance::path_manager ()
{

  e_WP = w - NEDp;

  uav_error.wp_dist = (uint16_t) norm (e_WP, 2);		// Global variable

  //cout << uav_error.wp_dist << endl;

  //Detects if wp has been missed
  //  if (uav_error.wp_dist < min_wp_dist || min_wp_dist == 0)
  //    {
  //      min_wp_dist = uav_error.wp_dist;
  //    }
  //  else if ((uav_error.wp_dist - min_wp_dist) >= WP_MISS_TOL)
  //    {
  //      wp_missed = true;
  //      string_convert << CONSOLE_ENTRY_NUMBER++ << " - WP missed " << CurrentWP + 1 << ".  Moving on.";
  //      sendToConsole ();
  //    }
  //  else
  //    {
  //      wp_missed = false;
  //    }

  //    printf("WP = %d\n ", CurrentWP);
  // Calculate r The waypoint that the sircraft just passed
  r (0) = WPList[CurrentWP].n;
  r (1) = WPList[CurrentWP].e;
  r (2) = WPList[CurrentWP].d;

  // The waypoint that the aircraft is trying to reach
  w (0) = WPList[CurrentWP + 1].n;
  w (1) = WPList[CurrentWP + 1].e;
  w (2) = WPList[CurrentWP + 1].d;



  // The next waypoint in the mission plan.
  s (0) = WPList[CurrentWP + 2].n;
  s (1) = WPList[CurrentWP + 2].e;
  s (2) = WPList[CurrentWP + 2].d;
  // Calculate q as shown in figure 10.1

  // Algorithm 6 , Pg 193, Ref 2 ( Waypoint following with fillets)
  q = (w - r) / norm ((w - r), 2);		// Calculating the vector between the waypoints w and r
  qi = (s - w) / norm ((s - w), 2);		// Calculating the vector between the waypoints s and w

  if( (uav_error.wp_dist < WPList[CurrentWP + 1].tolerance || wp_missed || mission_changed ) && state == 1)
    {

      //        if (!mission_changed)
      //  	{
      //  	  CurrentWP++;
      //  	  gcs_comms.missionReached (CurrentWP);
      //  	}

      //Reset flags
      mission_changed = false;
      wp_missed = false;
      min_wp_dist = 0;

      // Eq 10.1 Ref 2
      X_q = atan2 (q (1), q (0));
      if (X_q < 0)
	{
	  X_q += 2 * M_PI;
	}
      nav_bearing = X_q;

      gcs_comms.missionCurrent (CurrentWP + 1);


    }

  Q = acos (-dot (q, qi));						// Calculating the angle between the vectors q and qi
  rf = WPList[CurrentWP + 1].tolerance / ( (1/sin(Q / 2) ) - 1);	// Calculating the radius of the fillet


  if (state == 1)
    {

      flag = 1;
      r_temp (0) = r (0);
      r_temp (1) = r (1);
      r_temp (2) = r (2);



      z1 = w - (rf / tan (Q / 2)) * q;			//

      H1 = dot ((NEDp - z1), q);			// Calculating the Halfplane to trigger the airplane to arc mode



      if (H1 > t1 &&  CurrentWP + 2  < WPList.size() )
	{
	  state = 2;

	}

    }
  else if (state == 2  &&  CurrentWP + 2  < WPList.size() )
    {

      flag = 2;
      cf = w - (rf / sin (Q / 2)) * ((q - qi) / norm ((q - qi), 2));		//Calculating the center of the fillet
      rho = rf;
      z2 = w + (rf / tan (Q / 2)) * qi;

      if ((q (0) * qi (1) - q (1) * qi (0)) > 0)
	{
	  lambda = 1;
	}
      else if ((q (0) * qi (1) - q (1) * qi (0)) < 0)
	{
	  lambda = -1;
	}
      H2 = dot ((NEDp - z2), qi);			// Calculating the Halfplane to trigger the airplane to line mode

      if (H2 > t2 )
	{
	  state = 1;

	  CurrentWP++;
	  wp_missed = true;


	}
    }



}

void
guidance::course_heading ()
{

  // state = 1 follow straight line mechanism
  if (state == 1)
    {

      // Ref 2, Chp-10, Alg-3, Line 9
      e_py = -sin (X_q) * (NEDp (0) - r (0)) + cos (X_q) * (NEDp (1) - r (1));
      uav_error.xtrack = e_py; 	//Populate the global variable

      cmd.euler_angle.psi = (X_q - (X_inf) * (2 / M_PI) * atan (k_path * e_py));

      // Corrects any INS that outputs heading from
      // -180 to 180 instead of 0 to 360 - Move to UAV state correction later
      if (uav.euler_angle.psi < 0)
	{
	  uav.euler_angle.psi += 2 * M_PI;
	}
      if (uav_error.psi >= M_PI)
	{
	  uav_error.psi -= 2 * M_PI;
	}
      else if (uav_error.psi < -M_PI)
	{
	  uav_error.psi += 2 * M_PI;
	}
    }

  // state = 2 follow orbit mechanism
  else if (state == 2)
    {

      cd = sqrt (pow ((NEDp (0) - cf (0)), 2) + pow ((NEDp (1) - cf (1)), 2));
      phi_co = atan2 ((NEDp (1) - cf (1)), (NEDp (0) - cf (0))) + 2*M_PI;

      if (phi_co - ypr.c0 * (M_PI / 180) <= -M_PI)
	{
	  phi_co += 2 * M_PI;
	}
      else if (phi_co - ypr.c0 * (M_PI / 180) >= M_PI)
	{
	  phi_co -= 2 * M_PI;
	}

      cmd.euler_angle.psi = phi_co + lambda * ((M_PI / 2) + atan2 (k_orbit *(cd - rho) , rho));

    }

}

vec
guidance::projection_error (vec u, vec v)
{
  return u - dot (u, v / norm (v, 2)) * v / norm (v, 2);
}

void
guidance::position_error ()
{

  // Take in the NED location to the local
  // Armadillo vector
  NEDp (0) = uav.ned_pos.n;
  NEDp (1) = uav.ned_pos.e;
  NEDp (2) = (double) -altitude;

  e_path = projection_error (r - NEDp, q);

  // Send local variables to the global variables
  uav_error.n = e_path (0);
  uav_error.e = e_path (1);
  uav_error.d = e_path (2);
  uav_error.alt = -e_path (2);

  //cout << "error n = " << uav_error.n << " error e = " << uav_error.e <<" error d = " << uav_error.d << endl;

  // Throttle error
  e_airspeed = WPList[CurrentWP + 1].airspeed_cmd - airSpeed;
  uav_error.aspd = e_airspeed;

}

/////////////////////////////////////////////////

////////////////// Public ///////////////////////
void
guidance::init (float dt_in)
{

  dt = dt_in;

  // Initialize the path manager.
  CurrentWP = 0;
  gcs_comms.missionCurrent (CurrentWP + 1);

  // Initialize location variables
  NEDp (0) = uav.ned_pos.n;
  NEDp (1) = uav.ned_pos.e;
  NEDp (2) = uav.ned_pos.d;

  // Calculate r
  r (0) = WPList[CurrentWP].n;
  r (1) = WPList[CurrentWP].e;
  r (2) = WPList[CurrentWP].d;

  // The waypoint that the aircraft is trying to reach
  w (0) = WPList[CurrentWP + 1].n;
  w (1) = WPList[CurrentWP + 1].e;
  w (2) = WPList[CurrentWP + 1].d;

  s (0) = WPList[CurrentWP + 2].n;
  s (1) = WPList[CurrentWP + 2].e;
  s (2) = WPList[CurrentWP + 2].d;

  // Calculate q as shown in figure 10.1
  q = (w - r) / norm ((w - r), 2);
  qi = (s - w) / norm ((s - w), 2);
  // Eq 10.1 Ref 2
  X_q = atan2 (q (1), q (0));
  if (X_q < 0)
    {
      X_q += 2 * M_PI;
    }

  // Initialize PID objects
  if (DEBUG_LAT || DEBUG_LONG)
    {
      cout << "K_ALT_P = " << K_ALT_P << " K_ALT_I = " << K_ALT_I << " K_ALT_D = " << K_ALT_D << endl;
      cout << "K_XTRACK_P = " << K_XTRACK_P << " K_XTRACK_I = " << K_XTRACK_I << " K_XTRACK_D = " << K_XTRACK_D << endl;
      cout << "K_THROT_P = " << K_THROT_P << " K_THROT_I = " << K_THROT_I << " K_THROT_D = " << K_THROT_D << endl;
      cout << "LIMIT_THETA = " << LIMIT_THETA << " LIMIT_PHI = " << LIMIT_PHI << " LIMIT_THROTTLE = " << LIMIT_THROTTLE << endl;
      cout << "take_off : " << take_off << endl;
    }
  //Altitude.init (K_ALT_P, K_ALT_I, K_ALT_D, LIMIT_THETA, 0, dt);
  //heading2bank.init (K_XTRACK_P, K_XTRACK_I, K_XTRACK_D, LIMIT_PHI, 0, dt);
  //airspeed.init (K_THROT_P, K_THROT_I, K_THROT_D, LIMIT_THROTTLE, 0, dt);
  //takeoff_heading.init (.0165, 0, 0.0001, .5, 0, dt);
  Altitude(omega_alt, eta_alt, K1_alt, K2_alt,f_hat_alt, b_hat_alt, GPParams_alt,frequency); // frequency = 1.0/dt // omega_alt, eta_alt, K1_alt, K2_alt,f_hat_alt, b_hat_alt, GPParams_alt,frequency should come as some input into the init function
  heading2bank(omega_h2b, eta_h2b, K1_h2b, K2_h2b,f_hat_h2b, b_hat_h2b, GPParams_h2b,frequency);																												// best to send the input as a struct
  airspeed(omega_va, eta_va, K1_va, K2_va,f_hat_va, b_hat_va, GPParams_va,frequency);
  takeoff_heading(omega_takeoff, eta_takeoff, K1_takeoff, K2_takeoff,f_hat_takeoff, b_hat_takeoff, GPParams_takeoff,frequency);

}

void
guidance::update_attitude_cmd ()
{

  /* Compute path, error and course heading angle */
  position_error ();
  path_manager ();
  course_heading ();

  /* State Machine based on waypoint type/command (takeoff, landing, etc.) */
  switch (WPList[CurrentWP].command)
  {

    /* Typical straight path point to point navigation */
    case MAV_CMD_NAV_WAYPOINT:
      {
	cmd.euler_angle.phi = heading2bank.controller_output ( // Done 6_21 to fix erratic behavior
	    uav_error.psi, 0); // Update the heading to bank discrete pid loop - Ref 2, Eq. 6.3.2
	cmd.euler_angle.theta = Altitude.controller_output (uav_error.alt, 0); // Update the altitude discrete pid loop
	throttle_cmd = airspeed.controller_output (e_airspeed, 0);
	break;
      }
      /* Typical straight path point to point navigation */
    case MAV_CMD_NAV_TAKEOFF:
      {
	if (airSpeed < V_stall * 1.2)
	  { // Behavior before reaching stall speed

	    nosewheel_cmd = -takeoff_heading.controller_output (uav_error.psi, 0);
	    throttle_cmd = 1; // Full power on take off
	    cmd.euler_angle.phi = 0; // Maintain Level wings with respect to roll axis
	    cmd.euler_angle.theta = WPList[CurrentWP].param1 * M_PI / 180; // Take in the commanded theta for take-off

	  }
	else
	  {	// Change behavior to "point to point" type flight

	    take_off = false;
	    nosewheel_cmd = takeoff_heading.controller_output (0, true); // Reset the takeoff_heading pid to zeros
	    cmd.euler_angle.phi = heading2bank.controller_output ( // Done 6_21 to fix erratic behavior
		uav_error.psi, 0); // Update the heading to bank discrete pid loop - Ref 2, Eq. 6.3.2
	    cmd.euler_angle.theta = Altitude.controller_output (uav_error.alt, 0); // Update the altitude discrete pid loop
	    throttle_cmd = airspeed.controller_output (e_airspeed, 0);

	  }
	break;
      }

    case MAV_CMD_NAV_LAND:
      {
	if (abs (NEDp (2)) > 30)
	  {
	    cmd.euler_angle.phi = heading2bank.controller_output (uav_error.psi, 0); // Update the heading to bank discrete pid loop - Ref 2, Eq. 6.3.2
	    cmd.euler_angle.theta = Altitude.controller_output (uav_error.alt, 0); // Update the altitude discrete pid loop
	    throttle_cmd = .3;
	  }
	else
	  {
	    cmd.euler_angle.phi = heading2bank.controller_output (uav_error.psi, 0); // Update the heading to bank discrete pid loop - Ref 2, Eq. 6.3.2
	    cmd.euler_angle.theta = 7 * M_PI / 180;
	    throttle_cmd = 0;
	  }

	break;
      } // end MAV_CMD_NAV_LAND

    case MAV_CMD_DO_JUMP:
      {
	if (do_jump_count < WPList[CurrentWP].param2)
	  {
	    CurrentWP = WPList[CurrentWP].param1;
	    do_jump_count++;
	  }
	else
	  {
	    CurrentWP++;
	  }
	break;
      } // end MAV_CMD_DO_JUMP

    default:
      {
	/* Return to home - add functionality later*/
	break;
      }

  }

}

void
guidance::reset_to_runway ()
{
  mission_changed = true;
  CurrentWP = 0;
  take_off = true;
  gcs_comms.missionCurrent (CurrentWP + 1);
  string_convert << CONSOLE_ENTRY_NUMBER++ << " - Mission Reset";
  sendToConsole ();
}

//Class Destructor
guidance::~guidance ()
{
}
