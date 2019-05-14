
/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <turtlebot_trajectory_functions/circle.h>
#include <turtlebot_trajectory_functions/serpentine.h>
#include <turtlebot_trajectory_functions/angled_straight.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <nav_msgs/Odometry.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <turtlebot_trajectory_generator/near_identity.h>
#include <trajectory_generator_ros_interface.h>

#include <memory>

namespace turtlebot_trajectory_functions
{
using namespace turtlebot_trajectory_generator;
  
class TrajectoryTester 
{
public:
  TrajectoryTester(ros::NodeHandle& nh, std::string& name) : nh_(nh), pnh_("~"), name_(name)
  {
  }
  ~TrajectoryTester(){
  };

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool init()
  {    
    button_subscriber_ = nh_.subscribe("/mobile_base/events/button", 10, &TrajectoryTester::buttonCB, this);

    odom_subscriber_ = nh_.subscribe("/odom", 1, &TrajectoryTester::OdomCB, this);
    trajectory_publisher_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("/turtlebot_controller/trajectory_controller/desired_trajectory", 10);
    path_publisher_ = nh_.advertise<nav_msgs::Path>("/desired_path",10);
    
    nav_msgs::OdometryPtr init_odom(new nav_msgs::Odometry);

    curOdom_ = init_odom;
    
    nh_.param<std::string>("/mobile_base/base_frame", base_frame_id_, "base_footprint");

    return true;
  };

private:
  typedef ni_state state_type;
  typedef ni_controller traj_func_type;
  typedef trajectory_generator::trajectory_states<state_type, traj_func_type> traj_type;
  typedef std::shared_ptr<traj_type> traj_type_ptr;

private:
  trajectory_generator::TrajectoryGeneratorBridge<state_type, traj_func_type>  traj_gen_bridge;
  ros::NodeHandle nh_, pnh_;
  std::string name_;
  std::string base_frame_id_;
  ros::Subscriber button_subscriber_, odom_subscriber_;
  ros::Publisher trajectory_publisher_, path_publisher_;
  nav_msgs::OdometryPtr curOdom_;

  void buttonCB(const kobuki_msgs::ButtonEventPtr& msg);

  void OdomCB(const nav_msgs::OdometryPtr& msg);

  void generate_circle_trajectory();
  void generate_serpentine_trajectory();
  void generate_straight_trajectory();
  void make_trajectory(desired_traj_func::Ptr& dtraj);  
};

void TrajectoryTester::buttonCB(const kobuki_msgs::ButtonEventPtr& msg)
{
  if (msg->button == kobuki_msgs::ButtonEvent::Button0 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
  {
    ROS_INFO_STREAM("Button pressed: sending circle trajectory");

    TrajectoryTester::generate_circle_trajectory();
  }
  else
  if (msg->button == kobuki_msgs::ButtonEvent::Button1 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
  {
    ROS_INFO_STREAM("Button pressed: sending serpentine trajectory");

    TrajectoryTester::generate_serpentine_trajectory();
  }
  else
  if (msg->button == kobuki_msgs::ButtonEvent::Button2 && msg->state == kobuki_msgs::ButtonEvent::RELEASED )
  {
    ROS_INFO_STREAM("Button pressed: sending straight trajectory");
    
    TrajectoryTester::generate_straight_trajectory();
  }
  else
  {
    ROS_INFO_STREAM("Button event");
  }
};


void TrajectoryTester::generate_circle_trajectory()
{
    double fw_vel = .05;
    double r = .5;

    if(ros::param::get("~radius", r))
    {
      ROS_INFO_STREAM("Radius: " << r);
    }
    
    if(ros::param::get("~fw_vel", fw_vel))
    {
      ROS_INFO_STREAM("Fv_vel: " << fw_vel);
    }

    desired_traj_func::Ptr des_traj = std::make_shared<Circle>(fw_vel,r);
    make_trajectory(des_traj);
}

void TrajectoryTester::generate_straight_trajectory()
{
  double fw_vel = .05;
  double angle = .3;
  
  if(ros::param::get("~angle", angle))
  {
    ROS_INFO_STREAM("angle: " << angle);
  }
  
  if(ros::param::get("~fw_vel", fw_vel))
  {
    ROS_INFO_STREAM("fw_vel: " << fw_vel);
  }

  desired_traj_func::Ptr des_traj = std::make_shared<AngledStraight>(angle, fw_vel);
  make_trajectory(des_traj);
}



void TrajectoryTester::generate_serpentine_trajectory()
{
  double fw_vel = .05;
  double period = 3;
  double mag=1;
  
  if(ros::param::get("~magnitude", mag))
  {
    ROS_INFO_STREAM("magnitude: " << mag);
  }
  
  if(ros::param::get("~fw_vel", fw_vel))
  {
    ROS_INFO_STREAM("fw_vel: " << fw_vel);
  }
  
  if(ros::param::get("~period", period))
  {
    ROS_INFO_STREAM("period: " << period);
  }

  desired_traj_func::Ptr des_traj = std::make_shared<Serpentine>(fw_vel,mag,period);
  make_trajectory(des_traj);
}

void TrajectoryTester::make_trajectory(desired_traj_func::Ptr& dtraj)
{
  double v_max=.5;
  double w_max=4;
  double a_max=.55;
  double w_dot_max=1.78;
  
  near_identity ni(1,5,1,.01,v_max,w_max,a_max,w_dot_max);    
  traj_func_type::Ptr nc=std::make_shared<traj_func_type>(ni);
  nc->setTrajFunc(dtraj);
  
  double tf=10;
  
  
  auto odom = curOdom_;
  
  traj_type_ptr traj = std::make_shared<traj_type>();
  traj->header.frame_id = odom->child_frame_id;
  traj->header.stamp = odom->header.stamp;
  traj->trajpntr = nc ;
  traj->params = std::make_shared<trajectory_generator::traj_params>();
  traj->params->tf=tf;
  traj->x0_.from(odom->twist.twist);
  traj->x0_.lambda=.3;

  
  traj_gen_bridge.generate_trajectory(traj);
  
  ROS_INFO_STREAM("Size: " << traj->x_vec.size());
  
  std::cout << "Time" << '\t' << "Error" << '\t' << 'x' << '\t' << 'y' << '\t' << "theta" << '\t' << 'v' << '\t' << 'w' << '\t' << "lambda" << '\t' << "xd" << '\t' << "yd" << std::endl;
  
  for( size_t i=0; i < traj->num_states(); i++ )
  {
    state_type& state = traj->x_vec[i];
    
    double error_x = state.x-state.xd;
    double error_y = state.y-state.yd;
    
    double error = sqrt(error_x*error_x + error_y*error_y);
    std::cout << std::fixed << std::setw(4) <<std::setprecision(4) << traj->times[i] << "\t" << error << "\t" << state.x << "\t" << state.y << "\t" << state.theta << "\t" << state.v << "\t" << state.w << "\t" << state.lambda << "\t" << state.xd << "\t" << state.yd << std::endl;
  }
  
  pips_trajectory_msgs::trajectory_points trajectory_msg = traj->toMsg ();
  trajectory_publisher_.publish(trajectory_msg);
  
  nav_msgs::Path::ConstPtr path_msg = traj->toPathMsg();    
  path_publisher_.publish(path_msg);
}



void TrajectoryTester::OdomCB(const nav_msgs::OdometryPtr& msg)
{

  curOdom_ = msg;
}

} //namespace turtlebot_trajectory_functions

//http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_trajectory_sender"); //, ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName();
    turtlebot_trajectory_functions::TrajectoryTester tester(nh,name);
    
    //signal(SIGINT, mySigintHandler);

    if (tester.init())
    {
        ros::spin();
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise test_trajectory_sender!");
    }

    ROS_INFO_STREAM("Program exiting");
    return 0;

}

