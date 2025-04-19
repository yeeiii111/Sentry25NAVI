#include "navigation/fieldoptimizer.hpp"
void Field_Optimizer::setparam(const Params& param){
    param_ = param;
}
Eigen::Vector2d Field_Optimizer::force_calculate(const geometry_msgs::PoseStamped& pose, const ObstacleResult& obstacle)
{
    Eigen::Vector2d force;
    Eigen::Vector2d delta(pose.pose.position.x - obstacle.world_position.x, pose.pose.position.y - obstacle.world_position.y);
    double dist = delta.norm();
    if (dist > param_.safe_distance) {
        force = Eigen::Vector2d(0,0);
    }

    // 防止除以零
    double epsilon = 1e-3;
    dist = std::max(dist, epsilon);
    double term = 1/dist - 1/param_.safe_distance;
    double f_norm = param_.repulsive_gain * pow(term,2);
    force = delta.normalized() * f_norm;
    return force;
}
Eigen::Vector2d Field_Optimizer::attractive_calculate(const geometry_msgs::PoseStamped& tar_pose, const geometry_msgs::PoseStamped& cur_pose)
{
    Eigen::Vector2d force;
    Eigen::Vector2d delta(tar_pose.pose.position.x - cur_pose.pose.position.x, tar_pose.pose.position.y - cur_pose.pose.position.y);
    double dist = delta.norm();

    double f_norm = param_.attractive_gain * pow(dist ,2);
    force = delta.normalized() * f_norm;
    return force;    
}

bool Field_Optimizer::optimize(const std::vector<ObstacleResult>& result, const geometry_msgs::PoseStamped& pose, geometry_msgs::PoseStamped& pose_opt, double ratio)
{
    int iter = 0;
    pose_opt = pose;
    while(iter < param_.max_iterations )
    {
        Eigen::Vector2d total_force(0,0), step;
        double step_norm;
        for(auto &obs : result)
        {
            total_force += force_calculate(pose_opt, obs);
        }
        total_force += attractive_calculate(pose,pose_opt);
        step_norm = std::min(param_.max_step, total_force.norm());
        // std::cout << total_force.norm() << std::endl;

        step = total_force.normalized() * step_norm;
        pose_opt.pose.position.x += step.x();
        pose_opt.pose.position.y += step.y();

        if(step_norm < param_.convergence_thresh) 
        {
            pose_opt.pose.position.x = (pose_opt.pose.position.x -pose.pose.position.x )*ratio + pose.pose.position.x;
            pose_opt.pose.position.y = (pose_opt.pose.position.y -pose.pose.position.y )*ratio + pose.pose.position.y;
            return true;
        }

        iter ++;
    }
    return false;
}
void Field_Optimizer::GlobalPathCallback(const nav_msgs::PathConstPtr & msg)
{
    global_path = *msg;
}
