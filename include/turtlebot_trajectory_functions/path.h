#ifndef TURTLEBOT_TRAJECTORY_FUNCTIONS_PATH_h
#define TURTLEBOT_TRAJECTORY_FUNCTIONS_PATH_h

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%

#include <boost/math/interpolators/cubic_b_spline.hpp>

#include <turtlebot_trajectory_generator/near_identity.h>

#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/convert.h>
#include <tf2_path_msg/tf2_path_msg.h>

namespace turtlebot_trajectory_functions
{

/* The rhs of x' = f(x) defined as a class */
class Path : public virtual desired_traj_func{
  
  
public:
  Path(const nav_msgs::PathConstPtr& path)
  {     
    if(path)
    {
      construct(path->poses);
    }
  }
  
  Path(const nav_msgs::Path& path)
  {     
    construct(path.poses);
  }
  
  //TODO: implement 'init' function and use state to provide values for left_endpoint derivatives
  
  void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
  {
    dxdt.xd = spline_x.prime(t);
    dxdt.yd = spline_y.prime(t);
    //ROS_INFO_STREAM("dxdt: " << dxdt.xd << ", dydt: " << dxdt.yd);
  }
  
  double getTF()
  {
    return tf;
  }
  
private:
  void construct(const std::vector<geometry_msgs::PoseStamped>& poses)
  {
    double grid_spacing = .05;
    double v_des=.3;
    double step = grid_spacing / v_des;
    std::vector<double>x,y;
    
    tf=poses.size();
    
    for(const geometry_msgs::PoseStamped& pose : poses)
    {
      x.push_back(pose.pose.position.x);
      y.push_back(pose.pose.position.y);
    }
    spline_x = boost::math::cubic_b_spline<double>(x.begin(),x.end(),0,step,0,0); //Assuming start from rest and end at rest
    spline_y = boost::math::cubic_b_spline<double>(y.begin(),y.end(),0,step,0,0);
    
  }
  
private:
  boost::math::cubic_b_spline<double> spline_x, spline_y;
  double tf;
};
//]

} //namespace turtlebot_trajectory_functions

#endif //TURTLEBOT_TRAJECTORY_FUNCTIONS_PATH_h
 
 

