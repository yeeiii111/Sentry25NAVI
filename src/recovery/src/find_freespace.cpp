#include <pluginlib/class_list_macros.h>
#include "recovery/goal_reset.hpp"
#include "recovery/find_freespace.hpp"
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
PLUGINLIB_EXPORT_CLASS(recovery::FindFreespace,nav_core::RecoveryBehavior)

namespace recovery{

FindFreespace::FindFreespace(): nh_("~"){}
FindFreespace::~FindFreespace(){}

void FindFreespace::initialize(std::string name, tf2_ros::Buffer* tf, 
                  costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap)
{
        name_ = name;
        local_costmap_ = local_costmap;
        global_costmap_ = global_costmap;

        nh_.param<double>("max_search_radius", max_r, 0.3);
        nh_.param<double>("min_search_radius", min_r, 0.05);
        nh_.param<double>("speed",speed, 0.5);
        cmd_pub = nh_.advertise<geometry_msgs::Twist>("/escape_vel",10);
        marker_pub = nh_.advertise<visualization_msgs::Marker>("escape_goal", 10);
        ROS_INFO("1111111");
}
void FindFreespace::runBehavior()
{
    costmap_2d::Costmap2D *costmap;
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getCostmap()->getMutex()));
            costmap = global_costmap_->getCostmap();
    }

    auto size_x = costmap->getSizeInCellsX();
    auto size_y = costmap->getSizeInCellsY();
    auto resolution = costmap->getResolution();
    auto origin_x = costmap->getOriginX();
    auto origin_y = costmap->getOriginY();
    int grid_radius = max_r / resolution;
    double w_x ;
    double w_y ;
    int grid_x, grid_y;
    geometry_msgs::PoseStamped robot_pose;

    global_costmap_->getRobotPose(robot_pose);
    w_x = robot_pose.pose.position.x;
    w_y = robot_pose.pose.position.y;

    std::vector<geometry_msgs::Point> lethal_obs;
    std::vector<geometry_msgs::Point> freespace;
    if(!world2Grid(w_x,w_y,costmap,grid_x,grid_y)){
        ROS_ERROR("EMPTY COSTMAP!");
        return;
    }
    ROS_INFO(" F GRID X %d GRID y %d", grid_x, grid_y);
    ROS_INFO(" F WORLD X %f WORLD y %f", w_x, w_y);
    ROS_INFO(" F COST %d", costmap->getCost(grid_x, grid_y) );
    if(costmap->getCost(grid_x + 1, grid_y ) == costmap_2d::FREE_SPACE &&
    costmap->getCost(grid_x - 1, grid_y ) == costmap_2d::FREE_SPACE &&
    costmap->getCost(grid_x , grid_y + 1) == costmap_2d::FREE_SPACE &&
    costmap->getCost(grid_x , grid_y -1 ) == costmap_2d::FREE_SPACE 
)
    {
        ROS_INFO("Do not need to escape from obstacle");
        return;
    }

    for (int i = 0; i <= grid_radius; i ++)
    {
        for(int x =  grid_x - i; x <= grid_x + i; x ++)
            for(int y = grid_y - i; y <= grid_y + i; y++)
            {
                if (x < 0 || x >= size_x || y < 0 || y >= size_y) continue;
                if(costmap->getCost(x,y) == costmap_2d::FREE_SPACE) 
                {
                    geometry_msgs::Point point;
                    point.x = x;
                    point.y = y;
                    point.z = 0;
                    freespace.push_back(point);
                }
                if(costmap->getCost(x,y) == costmap_2d::LETHAL_OBSTACLE) 
                {
                    geometry_msgs::Point point;
                    point.x = x;
                    point.y = y;
                    point.z = 0;
                    lethal_obs.push_back(point);
                }
            }
    }
    if(freespace.empty()){
        ROS_ERROR("F FAILED TO FIND FREESPACE");
        return;
    }
    //calculate obs center
    double x_sum = 0, y_sum = 0;
    double x_center, y_center;
    geometry_msgs::Point new_goal_grid;
    if(!lethal_obs.empty())
    {
        for(auto&point : lethal_obs)
        {
            x_sum += point.x;
            y_sum += point.y;
        }
        x_center = x_sum / lethal_obs.size();
        y_center = y_sum / lethal_obs.size();

    //find the farest freespace to obs center
        double max_dist = 0;
        for(auto &point : freespace)
        {
            double dist = std::hypot(point.x - x_center, point.y - y_center);
            if (dist > max_dist){
                max_dist = dist;
                new_goal_grid = point;
            }
        } 
    }
    else new_goal_grid = freespace.front();

    geometry_msgs::PoseStamped new_goal_world;
    new_goal_world.header.frame_id = robot_pose.header.frame_id;
    new_goal_world.header.stamp = ros::Time::now();
    new_goal_world.pose.orientation = robot_pose.pose.orientation;
    new_goal_world.pose.position.z = 0;
    Grid2world(new_goal_grid.x,new_goal_grid.y,costmap,new_goal_world.pose.position.x,new_goal_world.pose.position.y);

    //cmd_vel pub
    geometry_msgs::Twist cmd_vel;
    double yaw = tf2::getYaw(robot_pose.pose.orientation);
    double diff_angle = atan2(new_goal_world.pose.position.y - robot_pose.pose.position.y, 
                                new_goal_world.pose.position.x - robot_pose.pose.position.x);
    double vx_global = speed * cos(diff_angle);
    double vy_global = speed * sin(diff_angle);
    cmd_vel.linear.x = vx_global * cos(yaw) + vy_global * sin(yaw);
    cmd_vel.linear.y = -vx_global * sin(yaw) + vy_global * cos(yaw);
    ros::Time start = ros::Time::now();
    while ((ros::Time::now() - start).toSec() < 1.0) {
        cmd_pub.publish(cmd_vel);
        ros::Duration(0.01).sleep(); 
    }
    ROS_INFO("cmd_vel x = %f  y = %f", cmd_vel.linear.x , cmd_vel.linear.x);
    //visualization
    visualization_msgs::Marker marker;

    marker.header.frame_id = robot_pose.header.frame_id;        // 坐标系（需与RViz全局选项一致）
    marker.header.stamp = ros::Time::now();
    marker.ns = "escape_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;  // 类型（如球体）
    marker.action = visualization_msgs::Marker::ADD;  // 操作类型（ADD/MODIFY/DELETE）

    marker.pose.position = new_goal_world.pose.position;          
    marker.pose.orientation = new_goal_world.pose.orientation;      
    marker.scale.x = 0.05;                  // 尺寸（单位：米，根据类型调整参数）
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // 颜色与生命周期
    marker.color.b= 1.0;                  // RGB通道（0.0-1.0）
    marker.color.a = 1.0;                  // 透明度（0为完全透明）
    marker.lifetime = ros::Duration(50); 

    marker_pub.publish(marker);
}


}