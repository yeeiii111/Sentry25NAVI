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
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
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
struct ObstacleResult {
    geometry_msgs::Point world_position;  // 障碍物世界坐标
    int grid_x;                           // 障碍物栅格坐标X
    int grid_y;                           // 障碍物栅格坐标Y
    unsigned char cost;                   // 代价值（0-100）
};
class Controller {
public:
    Controller();
    ~Controller();
    void GlobalPathCallback(const nav_msgs::PathConstPtr & msg);
    void CostmapCallback(const nav_msgs::OccupancyGridConstPtr & msg);
    void DivergeCallback(const std_msgs::BoolConstPtr &msg);
    void MatchCallback(const std_msgs::BoolConstPtr &msg);
    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist);
    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                    const nav_msgs::Path& traj,
                    geometry_msgs::Twist& cmd_vel);
    bool world2Grid(double wx, double wy, 
                    const nav_msgs::OccupancyGrid& costmap, 
                    int& gx, int& gy);
    bool Passbility_check(nav_msgs::OccupancyGrid &costmap, nav_msgs::Path &plan, double search_radius, int threshold, std::vector<ObstacleResult> &result);
    void publishObstaclesGrid(const std::vector<ObstacleResult>& obstacles, double resolution);
    void Plan(const ros::TimerEvent& event);
    double YawErrorCal(const geometry_msgs::PoseStamped& robot_pose,
                    const geometry_msgs::PoseStamped& path_pose);
private:
    ros::Publisher cmd_vel_pub;
    ros::Publisher local_path_pub;
    ros::Publisher forsee_path_pub;
    ros::Publisher obstacle_pub;
    ros::Subscriber diverge_sub;
    ros::Subscriber match_sub;
    ros::Subscriber global_path_sub;
    ros::Subscriber costmap_sub;
    ros::Timer      plan_timer;

    std::shared_ptr<tf::TransformListener> tf_listener;

    nav_msgs::OccupancyGrid costmap;
    nav_msgs::Path global_path;
    bool diverge = false;
    bool plan = false;
    bool turn_state = false;
    bool debug_en;
    int prune_index = 0;
    int forsee_index = 0;
    int  narrow_threshold;
    double max_speed;
    double set_yaw_speed = 0;
    double search_radius;
    double p_value;
    double straight_p_value;
    double curve_p_value;
    double wz_p_value;
    double wz_d_value;
    double wz_const;
    double last_yaw_error;
    int straight_foresee_index;
    int curve_foresee_index;   
    int plan_freq;
    double goal_dist_tolerance;
    double turn_tolerance;
    double prune_ahead_dist;

    double yaw;
    std::vector<ObstacleResult> obstacle_result;
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