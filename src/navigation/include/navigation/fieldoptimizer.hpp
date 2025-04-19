#ifndef FIELD_OPTIMIZER_H
#define FIELD_OPTIMIZER_H
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
struct ObstacleResult {
    geometry_msgs::Point world_position;  // 障碍物世界坐标
    int grid_x;                           // 障碍物栅格坐标X
    int grid_y;                           // 障碍物栅格坐标Y
    unsigned char cost;                   // 代价值（0-100）
};
class Field_Optimizer{
public:
    struct Params {
        double repulsive_gain = 0.02;   // 斥力增益系数
        double attractive_gain = 0.2;   // 目标点吸引力
        double safe_distance = 0.2;    // 安全距离(m)
        double max_step = 0.1;         // 最大单步移动距离
        double convergence_thresh = 0.01; // 收敛阈值
        int max_iterations = 100;      // 最大迭代次数
    };
    Field_Optimizer(){};
    Field_Optimizer(const Params param): param_(param){}
    ~Field_Optimizer(){};
    void setparam(const Params& param);
    void GlobalPathCallback(const nav_msgs::PathConstPtr & msg);
    bool optimize(const std::vector<ObstacleResult>& result, const geometry_msgs::PoseStamped& pose, geometry_msgs::PoseStamped& pose_opt, double ratio);
private:
    Eigen::Vector2d attractive_calculate(const geometry_msgs::PoseStamped& tar_pose, const geometry_msgs::PoseStamped& cur_pose);
    Eigen::Vector2d force_calculate(const geometry_msgs::PoseStamped& pose, const ObstacleResult& obstacle);
    Params param_;
    nav_msgs::Path  global_path;
    ros::Publisher optpath_pub;
    ros::Subscriber globalpath_sub;
    ros::Timer      opt_timer;
};
#endif