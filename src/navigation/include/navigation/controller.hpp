#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <mutex>
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
#include"navigation/fieldoptimizer.hpp"
class Controller {
public:
    Controller();
    ~Controller();
    void GlobalPathCallback(const nav_msgs::PathConstPtr & msg);
    void CostmapCallback(const nav_msgs::OccupancyGridConstPtr & msg);
    void MatchCallback(const std_msgs::BoolConstPtr &msg);
    void LocalizeCallback(const std_msgs::BoolConstPtr &msg);
    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist);
    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                    const nav_msgs::Path& traj,
                    geometry_msgs::Twist& cmd_vel);
    int Passbility_check(nav_msgs::OccupancyGrid &costmap, nav_msgs::Path &plan, double search_radius, 
                          std::vector<ObstacleResult> &result, int forsee_index);// = short_forsee_index
    void publishObstaclesGrid(const std::vector<ObstacleResult>& obstacles, double resolution, ros::Publisher& pub);
    void Plan(const ros::TimerEvent& event);
    void PathOptimaze(const ros::TimerEvent& event);
    double YawErrorCal(const geometry_msgs::PoseStamped& robot_pose,
                      const geometry_msgs::PoseStamped& path_pose);
    double CurvatureCal(const nav_msgs::Path& traj);
    double YawControl(const geometry_msgs::PoseStamped& robot_pose,
                      const geometry_msgs::PoseStamped& path_pose);
    double YawControl(double error);
private:
    ros::Publisher cmd_vel_pub;
    ros::Publisher prune_path_pub;
    ros::Publisher local_path_pub;
    ros::Publisher forsee_path_pub;
    ros::Publisher obstacle_pub;
    ros::Publisher narrow_pub;
    ros::Subscriber diverge_sub;
    ros::Subscriber match_sub;
    ros::Subscriber localize_success_sub;
    ros::Subscriber global_path_sub;
    ros::Subscriber costmap_sub;
    ros::Timer      plan_timer;
    ros::Timer      optimize_timer;

    std::chrono::_V2::system_clock::time_point localized_time;
    std::shared_ptr<tf::TransformListener> tf_listener;
    std_msgs::Bool  narrow_msg;
    nav_msgs::OccupancyGrid costmap;
    nav_msgs::Path global_path;
    nav_msgs::Path prune_path;
    nav_msgs::Path opt_path;
    bool narrow = false;
    bool diverge = false;
    bool localized = false;
    bool plan = false;
    bool turn_state = false;
    bool debug_en;
    int prune_index = 0;
    int follow_index = 0;
    int forsee_index = 0;
    int short_forsee_index = 0;
    int  narrow_threshold;
    double max_speed;
    double set_yaw_speed = 0;
    double search_radius;
    double p_value;
    double straight_p_value;
    double curve_p_value;
    double wz_p_value;
    double wz_d_value;
    double last_yaw_error;
    double curvature;
    int straight_foresee_index;
    int curve_foresee_index;   
    int plan_freq;
    int opt_freq;
    double goal_dist_tolerance;
    double prune_ahead_dist;

    double yaw;
    std::vector<ObstacleResult> obstacle_result;
    std::string global_frame;
    std::mutex  prunepath_mutex;
    std::mutex  optpath_mutex;
    Field_Optimizer::Params param_;
    Field_Optimizer optimizer_;

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
bool world2Grid(double wx, double wy, 
                const nav_msgs::OccupancyGrid& costmap, 
                int& gx, int& gy);
bool Grid2world(int gx, int gy, const nav_msgs::OccupancyGrid& costmap,
                double& wx, double& wy);
#endif