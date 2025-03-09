#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include"geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/JointState.h"

#include "utility.hpp"
#include <tf2/utils.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "cubic_spline/cubic_spline_ros.h"
#include <Eigen/Eigen>
#include <chrono>
#include <std_msgs/Bool.h>
class Controller {
public:
    Controller();
    ~Controller();
    void GlobalPathCallback(const nav_msgs::PathConstPtr & msg);
    void LocalizationStatusCallback(const std_msgs::BoolConstPtr &msg);
    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist);
    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                    const nav_msgs::Path& traj,
                    geometry_msgs::Twist& cmd_vel);
    void Plan(const ros::TimerEvent& event);
private:
    ros::Publisher cmd_vel_pub;
    ros::Publisher local_path_pub;
    ros::Subscriber act_command_sub;
    ros::Subscriber global_path_sub;
    ros::Timer      plan_timer;

    std::shared_ptr<tf::TransformListener> tf_listener;

    nav_msgs::Path global_path;
    bool diverge = false;
    bool plan = false;
    int prune_index = 0;

    double max_speed;
    double set_yaw_speed = 0;

    double p_value;

    int plan_freq;
    double goal_dist_tolerance;
    double prune_ahead_dist;

    double yaw;
    std::string global_frame;

};   
double normalizeRadian(const double angle)
{
   double n_angle = std::fmod(angle, 2 * M_PI);
   n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
   return n_angle;
}

double ABS_limit(double value,double limit)
{
  if(value<limit && value>-limit)
  {
    return 0;
  }
  else
  {
    return value;
  }

}
#endif