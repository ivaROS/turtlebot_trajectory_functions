#ifndef TURTLEBOT_TRAJECTORY_FUNCTIONS_ANGLED_STRAIGHT_H
#define TURTLEBOT_TRAJECTORY_FUNCTIONS_ANGLED_STRAIGHT_H

#include <turtlebot_trajectory_generator/near_identity.h>

namespace turtlebot_trajectory_functions
{


/* The rhs of x' = f(x) defined as a class */
class AngledStraight : public desired_traj_func{

    double dep_angle_;
    double v_;

public:
    AngledStraight( double dep_angle, double v ) : dep_angle_(dep_angle), v_(v) { }
    
    void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
    {
        dxdt[ni_state::XD_IND] = v_*cos( dep_angle_);
        dxdt[ni_state::YD_IND] = v_*sin( dep_angle_);
    }
    
    
};

} //end namespace turtlebot_trajectory_functions

#endif //TURTLEBOT_TRAJECTORY_FUNCTIONS_ANGLED_STRAIGHT_H
