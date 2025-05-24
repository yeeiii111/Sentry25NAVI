#include <pluginlib/class_list_macros.h>
#include "recovery/goal_reset.hpp"
#include <visualization_msgs/Marker.h>
PLUGINLIB_EXPORT_CLASS(recovery::Goal_Reset,nav_core::RecoveryBehavior)

namespace recovery{

Goal_Reset::Goal_Reset():nh_("~"){};
Goal_Reset::~Goal_Reset(){};

void Goal_Reset::initialize(std::string name, tf2_ros::Buffer* tf, 
                  costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap)
{
    name_ = name;
    local_costmap_ = local_costmap;
    global_costmap_ = global_costmap;
    nh_.param<double>("max_search_radius", max_r, 0.3);
    nh_.param<double>("min_search_radius", min_r, 0.05);

    goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("new_goal", 10);
    goal_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,&Goal_Reset::GoalCallback,this);
}

void Goal_Reset::runBehavior()
{
    auto costmap = global_costmap_->getCostmap();
    auto size_x = costmap->getSizeInCellsX();
    auto size_y = costmap->getSizeInCellsY();
    auto resolution = costmap->getResolution();
    auto origin_x = costmap->getOriginX();
    auto origin_y = costmap->getOriginY();

    int grid_radius = max_r / resolution;
    double w_x = initial_goal.pose.position.x;
    double w_y = initial_goal.pose.position.y;
    int grid_x, grid_y;

    std::vector<geometry_msgs::Point> lethal_obs;
    std::vector<geometry_msgs::Point> freespace;

    if(!world2Grid(w_x,w_y,costmap,grid_x,grid_y)){
        ROS_ERROR("EMPTY COSTMAP!");
        return;
    }
    ROS_INFO("GRID X %d GRID y %d", grid_x, grid_y);
    ROS_INFO("WORLD X %f WORLD y %f", w_x, w_y);
    if(costmap->getCost(grid_x, grid_y) == costmap_2d::FREE_SPACE)
    {
        ROS_INFO("Do not need to reset goal");
        return;
    }
    for (int i = 0; i <= grid_radius; i ++)
    {
        for(int x = grid_x  - i; x <= grid_x + i; x ++)
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
        ROS_ERROR("FAILED TO FIND FREESPACE");
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
    new_goal_world.header.frame_id = initial_goal.header.frame_id;
    new_goal_world.header.stamp = ros::Time::now();
    new_goal_world.pose.orientation = initial_goal.pose.orientation;
    new_goal_world.pose.position = initial_goal.pose.position;
    if(Grid2world(new_goal_grid.x,new_goal_grid.y,costmap,new_goal_world.pose.position.x,new_goal_world.pose.position.y))
        goal_pub.publish(new_goal_world);

    //visualization
    visualization_msgs::Marker marker;

    marker.header.frame_id = initial_goal.header.frame_id;        // 坐标系（需与RViz全局选项一致）
    marker.header.stamp = ros::Time::now();
    marker.ns = "new_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;  // 类型（如球体）
    marker.action = visualization_msgs::Marker::ADD;  // 操作类型（ADD/MODIFY/DELETE）

    marker.pose.position = new_goal_world.pose.position;          
    marker.pose.orientation = new_goal_world.pose.orientation;      
    marker.scale.x = 0.03;                  // 尺寸（单位：米，根据类型调整参数）
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    // 颜色与生命周期
    marker.color.r = 1.0;                  // RGB通道（0.0-1.0）
    marker.color.a = 1.0;                  // 透明度（0为完全透明）
    marker.lifetime = ros::Duration(5); 

    marker_pub.publish(marker);
}
void Goal_Reset::GoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    initial_goal = *msg;
}




}

