#ifndef TURTLEBOT_TRAJECTORY_FUNCTIONS_CIRCLE_H
#define TURTLEBOT_TRAJECTORY_FUNCTIONS_CIRCLE_H

#include <turtlebot_trajectory_generator/near_identity.h>

namespace turtlebot_trajectory_functions
{


/* The rhs of x' = f(x) defined as a class */
class Circle : public desired_traj_func{
    double vf_; //Forward vel
    double r_;  //radius of circle

public:
    Circle( double vf, double r) : vf_(vf), r_(r) { }
    
    void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
    {
        dxdt[ni_state::YD_IND] = vf_*sin( - (vf_/r_) * t );
        dxdt[ni_state::XD_IND] = vf_*cos( - (vf_/r_) * t );
    }
    
};

} //end namespace turtlebot_trajectory_functions

#endif //TURTLEBOT_TRAJECTORY_FUNCTIONS_CIRCLE_H
