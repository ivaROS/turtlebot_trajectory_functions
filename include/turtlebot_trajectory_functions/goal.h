#ifndef TURTLEBOT_TRAJECTORY_FUNCTIONS_GOAL_H
#define TURTLEBOT_TRAJECTORY_FUNCTIONS_GOAL_H

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%


#include <turtlebot_trajectory_functions/angled_straight.h>
#include <geometry_msgs/Pose.h>

namespace turtlebot_trajectory_functions
{

using namespace turtlebot_trajectory_generator;
  
/* The rhs of x' = f(x) defined as a class */
class Goal : public AngledStraight {
  
  
public:
  
  Goal(const geometry_msgs::Point& position, double v):
    AngledStraight(getDepAngle(position), v)
  {     
    construct(position, v);
  }
  
  double getTF()
  {
    return tf_;
  }
  
  typedef std::shared_ptr<Goal> Ptr;
  
private:
  void construct(const geometry_msgs::Point& position, double v)
  {
    double total_length= std::sqrt((position.x)*(position.x) + (position.y)*(position.y));
    tf_=total_length/v;
   
    ROS_DEBUG_STREAM("Total length: " << total_length << ", desired v: " << v << ", tf: " << tf_);
  }
  
  static double getDepAngle(const geometry_msgs::Point& position)
  {
    double dep_angle = std::atan2(position.y, position.x);
    return dep_angle;
  }
  
private:
  double tf_;
};
//]

} //namespace turtlebot_trajectory_functions

#endif //TURTLEBOT_TRAJECTORY_FUNCTIONS_GOAL_H
 
 

