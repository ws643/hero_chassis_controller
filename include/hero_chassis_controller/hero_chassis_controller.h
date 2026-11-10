#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>/*定位+区分同名文件*/
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include<control_toolbox/pid.h>
#include<std_msgs/Float64MultiArray.h>
#include<geometry_msgs/Twist.h>


namespace hero_chassis_controller 
{
  class HeroChassisController 
  : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    /*public继承，父类的PUBILC成员到子类还是变成public*/
  {
    public:
      bool init(hardware_interface::EffortJointInterface* effort_joint_interface , ros::NodeHandle& root_nh , ros::NodeHandle& controller_nh) override;
      void starting(const ros::Time& time) override;/*起始时间*/
      void update(const ros::Time& time, const ros::Duration& period) override;/*起始时间+距离上一次update的时间间隔*/ 
      private:
      hardware_interface::JointHandle joints_[4];
      control_toolbox::Pid pids_[4];
      double target_speed_[4] = {0.0,0.0,0.0,0.0};
      void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
      ros::Subscriber cmd_vel_sub_;
      double vx_ = 0.0;
      double vy_ = 0.0;
      double wz_ = 0.0;
      double half_length_sum_ = 0.0;
      double wheel_radius_ = 0.0;

      ros::Publisher debug_pub_;
    };
  }


  #endif
