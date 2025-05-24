#pragma once

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_layer.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace recovery{
class FindFreespace : public nav_core::RecoveryBehavior{
public:
    FindFreespace();
    ~FindFreespace();
    void initialize(std::string name, tf2_ros::Buffer* tf, 
                  costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap);
    void runBehavior();
    void GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
private:

    costmap_2d::Costmap2DROS* global_costmap_;
    costmap_2d::Costmap2DROS* local_costmap_;   

    std::string name_;
    ros::NodeHandle nh_;


    double max_r, min_r,speed;
    ros::Publisher cmd_pub;
    ros::Publisher marker_pub;

};
}