#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include<string>
#include<cmath>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hero_chassis_controller 
{

  bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    
    
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

    }                   /*controller_nh是一个工具包，起点路径是工具之一但是里面还有他自带的函数比如advertise , publish*/
  
   debug_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("debug",10);
   odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom",10);
   debug_pub1_ = controller_nh.advertise<std_msgs::Float64MultiArray>("debug1",10);
   double wheel_track = 0.0;
   double wheel_base = 0.0;
  
   if(!controller_nh.getParam("wheel_track", wheel_track) || !controller_nh.getParam("wheel_base", wheel_base) || !controller_nh.getParam("wheel_radius_", wheel_radius_))
   {
     ROS_ERROR("Failed to read wheel_track/wheel_base/wheel_radius");
     return false;
   }
 
   half_length_sum_ = (wheel_track + wheel_base) / 2.0;
   cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::CmdVelCallback, this);
   /*     整个机器人上想让底盘动的控制器都能给底盘发指令。先创建订阅者，再发话题（。。绑定回调函数，收到消息就运行这个函数  this指向当前这个控制器对象  */

   controller_nh.getParam("a", a);
   controller_nh.getParam("which_mode", which_mode_);
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
    vx_for_odom_ = msg->linear.x;
    vy_for_odom_ = msg->linear.y;
    wz_for_odom_ = msg->angular.z;
  }
  
  
  
  
  void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) 
  {
    double dt = period.toSec();

    double target_vx = vx_for_odom_;
    double target_vy = vy_for_odom_;
    double target_wz = wz_for_odom_;

    if(which_mode_ == true)
    {
      target_vx = vx_for_odom_*cos(odom_yaw_) + vy_for_odom_*sin(odom_yaw_);
      target_vy = -vx_for_odom_*sin(odom_yaw_) + vy_for_odom_*cos(odom_yaw_);
      target_wz = wz_for_odom_; 
    }
    
    if (target_vx >  10.0) 
    target_vx =  10.0;
    if (target_vy >  10.0)
    target_vy =  10.0;
    
    double addspeed = a * dt;

    if(a > 0.0)
    {
      if(vx_  < target_vx)
      {
        vx_ += addspeed;
        if(vx_ > target_vx)
        {
          vx_ = target_vx;
        }
      }
      else if(vx_  > target_vx)
      {
        vx_ -= addspeed;
        if(vx_ < target_vx)
        {
          vx_ = target_vx;
        }
      }
      if(vy_  < target_vy)
      {
        vy_ += addspeed;
        if(vy_ > target_vy)
        {
          vy_ = target_vy;
        }
      }
      else if(vy_  > target_vy)
      {
        vy_ -= addspeed;
        if(vy_ < target_vy)
        {
          vy_ = target_vy;
        }
      }
      wz_ = target_wz;
    }
    else
    {
      vx_ = target_vx;
      vy_ = target_vy;
      wz_ = target_wz;
    }

       
    const double l = half_length_sum_;
    target_speed_[0] = (vx_ - vy_ - l * wz_) / wheel_radius_;  
    target_speed_[1] = (vx_ + vy_ + l * wz_) / wheel_radius_;  
    target_speed_[2] = (vx_ + vy_ - l * wz_) / wheel_radius_;  
    target_speed_[3] = (vx_ - vy_ + l * wz_) / wheel_radius_;  
    std_msgs::Float64MultiArray debug;
    std_msgs::Float64MultiArray debug1;
    /*创建一个结构体变量名叫debug,这个结构体里有一个名叫data的vector数组和一个描述这个数组的结构体*/
    

     for (int i = 0; i < 4; ++i) 
     {
        double error = target_speed_[i] - joints_[i].getVelocity();  
        double effort = pids_[i].computeCommand(error, period);     
        /*内部实现了PID三项控制
          公式： 输出= kp*error + ki*积分 + kd*误差变化率 */
        joints_[i].setCommand(effort);                             
        debug.data.push_back(target_speed_[i]);
        debug.data.push_back(joints_[i].getVelocity());
     }
     debug_pub_.publish(debug);

    realvx_  = (joints_[0].getVelocity()  + joints_[1].getVelocity() + joints_[2].getVelocity() + joints_[3].getVelocity()) * wheel_radius_ / 4.0;
    realvy_  = (-joints_[0].getVelocity()  + joints_[1].getVelocity() + joints_[2].getVelocity() - joints_[3].getVelocity()) * wheel_radius_ / 4.0;
    realwz_  = (-joints_[0].getVelocity()  + joints_[1].getVelocity() - joints_[2].getVelocity() + joints_[3].getVelocity()) * wheel_radius_ / (4.0*half_length_sum_);

    debug1.data.push_back(realvx_);
    debug1.data.push_back(vx_);
    debug1.data.push_back(realvy_);
    debug1.data.push_back(vy_);
    debug1.data.push_back(realwz_);
    debug1.data.push_back(wz_);
    debug_pub1_.publish(debug1);

    odom_x_ += (realvx_*cos(odom_yaw_) - realvy_*sin(odom_yaw_)) * dt;
    odom_y_ += (realvx_*sin(odom_yaw_) + realvy_*cos(odom_yaw_)) * dt;
    odom_yaw_ += realwz_ * dt;
    
    nav_msgs::Odometry odom;
    
      odom.header.stamp = time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
    
      odom.pose.pose.position.x = odom_x_;
      odom.pose.pose.position.y = odom_y_;
      odom.pose.pose.position.z = 0.0;
    
      tf2::Quaternion msgs;
      msgs.setRPY(0.0, 0.0, odom_yaw_);
    
      odom.pose.pose.orientation = tf2::toMsg(msgs); 
    
      odom.twist.twist.linear.x = realvx_;
      odom.twist.twist.linear.y = realvy_;
      odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.z = realwz_;
    

    odom_pub_.publish(odom);

    geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";


      odom_trans.transform.translation.x = odom_x_;
      odom_trans.transform.translation.y = odom_y_;
      odom_trans.transform.translation.z = 0.0;
 
      odom_trans.transform.rotation = tf2::toMsg(msgs);

    transform_broadcaster_.sendTransform(odom_trans);
  }


 PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase);
}
