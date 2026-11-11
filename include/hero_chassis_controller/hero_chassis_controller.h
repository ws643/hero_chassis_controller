#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>/*定位+区分同名文件*/
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include<control_toolbox/pid.h>
#include<std_msgs/Float64MultiArray.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<tf2_ros/transform_broadcaster.h>


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
      double vx_for_odom_ = 0.0;
      double vy_for_odom_ = 0.0;
      double wz_for_odom_ = 0.0;

      double half_length_sum_ = 0.0;
      double wheel_radius_ = 0.0;
      double realvx_ = 0.0;
      double realvy_ = 0.0;
      double realwz_ = 0.0;
      double odom_x_ = 0.0;
      double odom_y_ = 0.0;
      double odom_z_ = 0.0;
      double odom_yaw_ = 0.0;
      uint32_t sum = 0.0;
      tf2_ros::TransformBroadcaster transform_broadcaster_;
      ros::Publisher odom_pub_;
      ros::Publisher debug_pub_;
      bool which_mode_ = false; 
      //false表示 ： 输入指令为base_link坐标系下的速度指令
      //true表示 ： 输入指令为odom坐标系下的速度指令
      double a =0.0 ;
    };
  }

  #endif
