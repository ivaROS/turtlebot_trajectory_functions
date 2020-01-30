#ifndef TURTLEBOT_TRAJECTORY_FUNCTIONS_PATH_H
#define TURTLEBOT_TRAJECTORY_FUNCTIONS_PATH_H

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

using namespace turtlebot_trajectory_generator;
  
/* The rhs of x' = f(x) defined as a class */
class Path : public desired_traj_func{
  
  
public:
  Path(const nav_msgs::PathConstPtr& path, double v)
  {     
    if(path)
    {
      construct(path->poses, v);
    }
  }
  
  Path(const nav_msgs::Path& path, double v)
  {     
    construct(path.poses, v);
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
  
  typedef std::shared_ptr<Path> Ptr;
  
private:
  void construct(const std::vector<geometry_msgs::PoseStamped>& poses, double v)
  {
    int num_poses = poses.size();
    if(num_poses < 3)
    {
      ROS_ERROR_STREAM("Path only has " << num_poses << " poses, it must have at least 3!");
      return; //Maybe throw an exception?
    }
    
    ROS_ASSERT_MSG(v>0, "Error! Cannot have v <=0!");
    
    std::vector<double>x,y;
    for(const geometry_msgs::PoseStamped& pose : poses)
    {
      x.push_back(pose.pose.position.x);
      y.push_back(pose.pose.position.y);
    }
    
    double total_length=0;
    for(int i=1; i < x.size(); ++i)
    {
      total_length += std::sqrt((x[i]-x[i-1])*(x[i]-x[i-1])+(y[i]-y[i-1])*(y[i]-y[i-1]));
    }
    
    {
      double spacing = std::sqrt((x[1]-x[0])*(x[1]-x[0])+(y[1]-y[0])*(y[1]-y[0]));
      double pose_step = spacing / v;
      double pose_tf = pose_step * x.size();
      ROS_DEBUG_STREAM("Pose spacing: " << spacing << ", desired v: " << v << ", step: " << pose_step << ", tf: " << pose_tf);
    }
    
    tf=total_length/v;
    double step = tf/x.size();
    
    
    ROS_DEBUG_STREAM("Total length: " << total_length << ", desired v: " << v << ", average spacing: " << total_length/x.size() << ", step: " << step << ", tf: " << tf);
    
    spline_x = boost::math::cubic_b_spline<double>(x.begin(),x.end(),0,step,0,0); //Assuming start from rest and end at rest
    spline_y = boost::math::cubic_b_spline<double>(y.begin(),y.end(),0,step,0,0);
    
  }
  
private:
  boost::math::cubic_b_spline<double> spline_x, spline_y;
  double tf;
};
//]

} //namespace turtlebot_trajectory_functions

#endif //TURTLEBOT_TRAJECTORY_FUNCTIONS_PATH_H
 
 

