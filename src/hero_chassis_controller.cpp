#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include<string>

namespace hero_chassis_controller {

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
       return true;    
}
 void HeroChassisController::starting(const ros::Time& time) 
  {
    for(int i = 0 ; i < 4 ; ++i)
    {
      pids_[i].reset();
    }
  }
void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) 
  {
    for(int i=0 ;i < 4 ;++i)
    {
      target_speed_[i] = 5.0;
    }

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
