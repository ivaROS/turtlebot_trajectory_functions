#ifndef TURTLEBOT_TRAJECTORY_FUNCTIONS_SERPENTINE_H
#define TURTLEBOT_TRAJECTORY_FUNCTIONS_SERPENTINE_H

#include <turtlebot_trajectory_generator/near_identity.h>

namespace turtlebot_trajectory_functions
{

/* The rhs of x' = f(x) defined as a class */
class Serpentine : public desired_traj_func{
  double vf_; //Forward vel
  double mag_;
  double period_;
  
public:
  Serpentine( double vf, double mag, double period) : vf_(vf), mag_(mag), period_(period) { }
  
  
  void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
  {
    double dx = std::sqrt(vf_*vf_/(1+mag_*mag_*period_*period_*std::cos(period_*x.xd-period_/2)*std::cos(period_*x.xd-period_/2)));
    dxdt[ni_state::YD_IND] = mag_*period_*std::cos(period_*x[ni_state::XD_IND]-period_/2) * dx;
    dxdt[ni_state::XD_IND] = dx;
    
    dxdt.xd = dx;
  }
  
  
};

} //end namespace turtlebot_trajectory_functions

#endif //TURTLEBOT_TRAJECTORY_FUNCTIONS_SERPENTINE_H
