#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include<string>

namespace hero_chassis_controller 
{

  bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    //  front_left_joint_  = effort_joint_interface -> getHandle("left_front_wheel_joint");
    //  front_right_joint_ = effort_joint_interface -> getHandle("right_front_wheel_joint");
    //  back_left_joint_   = effort_joint_interface -> getHandle("left_back_wheel_joint");
    //  back_right_joint_  = effort_joint_interface -> getHandle("right_back_wheel_joint"); 
    const char* joint_names[4] = {"left_front_wheel_joint" ,  "right_front_wheel_joint" ,
                                "left_back_wheel_joint"  ,  "right_back_wheel_joint"};

    const char* pid_names[4]   = { "front_left" , "front_right"  ,
                                 "back_left"  , "back_right"   };  
  
  
   for(int i = 0 ; i < 4 ; i++)
    {
        joints_[i] = effort_joint_interface -> getHandle(joint_names[i]);

        if(!pids_[i].init(ros::NodeHandle(controller_nh, std::string("pid/") + pid_names[i])))
        {
              ROS_ERROR("Failed to init PID for %s", pid_names[i]);
              return false;
        }

    }
  
   debug_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("debug",10);
   double wheel_track = 0.0;
   double wheel_base = 0.0;
   /*这两个为啥不写在头文件里？*/
   if(!controller_nh.getParam("wheel_track", wheel_track) || !controller_nh.getParam("wheel_base", wheel_base) || !controller_nh.getParam("wheel_radius_", wheel_radius_))
   {/*括号里的看不懂*/
     ROS_ERROR("Failed to read wheel_track/wheel_base/wheel_radius");/*为啥不用cout*/
     return false;
   }
 
   half_length_sum_ = (wheel_track + wheel_base) / 2.0;
   cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::CmdVelCallback, this);

   return true;

  } /*init结束*/

  void HeroChassisController::starting(const ros::Time& time) 
  {
    for(int i = 0 ; i < 4 ; ++i)
    {
      pids_[i].reset();
    }
    vx_ = 0.0;
    vy_ = 0.0;
    wz_ = 0.0;
  }
  

  /*######                回      调       函         数                         ######*/
  void HeroChassisController::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    wz_ = msg->angular.z;
  }



  
  void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) 
  {
    //  for(int i=0 ;i < 4 ;++i)
    //  {

    //   target_speed_[i] = 5.0;

    //  }
    const double l = half_length_sum_;
    target_speed_[0] = (vx_ - vy_ - l * wz_) / wheel_radius_;  // front left
    target_speed_[1] = (vx_ + vy_ + l * wz_) / wheel_radius_;  // front right
    target_speed_[2] = (vx_ + vy_ - l * wz_) / wheel_radius_;  // back left
    target_speed_[3] = (vx_ - vy_ + l * wz_) / wheel_radius_;  // back right
    std_msgs::Float64MultiArray debug;

     for (int i = 0; i < 4; ++i) 
     {
        double error = target_speed_[i] - joints_[i].getVelocity();  
        double effort = pids_[i].computeCommand(error, period);     
        joints_[i].setCommand(effort);                             
        debug.data.push_back(target_speed_[i]);
        debug.data.push_back(joints_[i].getVelocity());
     }
     debug_pub_.publish(debug);

  }


 PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase);
}
