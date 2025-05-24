#pragma once

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_layer.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace recovery{
class Goal_Reset : public nav_core::RecoveryBehavior{
public:
    Goal_Reset();
    ~Goal_Reset();
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

    double max_r, min_r;
    ros::Publisher goal_pub;
    ros::Publisher marker_pub;
    ros::Subscriber goal_sub;

    geometry_msgs::PoseStamped initial_goal;
};
bool world2Grid(
    double wx, double wy, 
    const costmap_2d::Costmap2D* costmap, 
    int& gx, int& gy) 
{

    if (costmap->getSizeInCellsX() == 0 || costmap->getSizeInCellsY()== 0) return false;

    gx = static_cast<int>((wx - costmap->getOriginX()) / costmap->getResolution());
    gy = static_cast<int>((wy - costmap->getOriginY()) / costmap->getResolution());

    return (gx >= 0 && gx < costmap->getSizeInCellsX() && gy >= 0 && gy < costmap->getSizeInCellsY());
}
bool Grid2world(int gx, int gy, const costmap_2d::Costmap2D* costmap,
                double& wx, double& wy)
{
    if (costmap->getSizeInCellsX() == 0 || costmap->getSizeInCellsY()== 0) return false;

    wx = (gx+0.5) * costmap->getResolution() + costmap->getOriginX();
    wy = (gy+0.5) * costmap->getResolution() + costmap->getOriginY();

    return true;   
}
}