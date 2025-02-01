#include"navigation/controller.hpp"

Controller::Controller()
{
    ros::NodeHandle nh("~");
    nh.param<double>("max_speed", max_speed, 1.0);
    nh.param<double>("p_value", p_value, 0.5);
    nh.param<double>("set_yaw_speed", set_yaw_speed, 0);
    nh.param<int>("plan_frequency", plan_freq, 30);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance, 0.2);
    nh.param<double>("prune_ahead_distance", prune_ahead_dist, 0.5);
    nh.param<std::string>("global_frame", global_frame, "map");
    
    local_path_pub = nh.advertise<nav_msgs::Path>("local_path",5);
    global_path_sub = nh.subscribe("/move_base/GlobalPlanner/plan",5,&Controller::GlobalPathCallback,this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    tf_listener = std::make_shared<tf::TransformListener>();
    plan_timer = nh.createTimer(ros::Duration(1.0/plan_freq),&Controller::Plan,this);

}

void Controller::Plan(const ros::TimerEvent& event){

    if(plan){
        auto start = ros::Time::now();
        geometry_msgs::PoseStamped robot_pose;
        GetTargetRobotPose(tf_listener, global_path.header.frame_id, robot_pose);
        if (GetEuclideanDistance(robot_pose,global_path.poses.back())<= goal_dist_tolerance
            || prune_index == global_path.poses.size() - 1){
            plan = false;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            cmd_vel_pub.publish(cmd_vel);
            ROS_INFO("Planning Success!");
            return;
        }

        FindNearstPose(robot_pose,global_path,prune_index, prune_ahead_dist);
        
        nav_msgs::Path prune_path, local_path;

        prune_path.header.frame_id = global_frame;
        prune_path.poses.push_back(robot_pose);

        int i = prune_index;
        while(i < global_path.poses.size() && i - prune_index < 20){
            prune_path.poses.push_back(global_path.poses[i]);
            i++;
        }

        GenTraj(prune_path, local_path);
        local_path_pub.publish(local_path);

        geometry_msgs::Twist cmd_vel;
        FollowTraj(robot_pose, local_path, cmd_vel);
        cmd_vel_pub.publish(cmd_vel);
    }   
    else{
        geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = set_yaw_speed;
            cmd_vel.linear.z = 0;   // bool success or not
            cmd_vel_pub.publish(cmd_vel);
    }
    

}

void Controller::GlobalPathCallback(const nav_msgs::PathConstPtr & msg){
  if (!msg->poses.empty()){
      global_path = *msg;
      prune_index = 0;
      plan = true;
  }
}

void Controller::FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist){

            double dist_threshold = 10;// threshold is 10 meters (basically never over 10m i suppose)
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist;
            if(prune_index!=0){
                sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index-1]);
            }else{
                sq_dist = 1e10;
            }

            double new_sq_dist = 0;
            while (prune_index < (int)path.poses.size()) {
                new_sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index]);
                if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {

                    //Judge if it is in the same direction and sq_dist is further than 0.3 meters
                    if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                        (path.poses[prune_index-1].pose.position.x - robot_pose.pose.position.x) +
                        (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                        (path.poses[prune_index-1].pose.position.y - robot_pose.pose.position.y) > 0
                        && sq_dist > prune_ahead_dist) {
                        prune_index--;
                    }else{
                        sq_dist = new_sq_dist;
                    }

                    break;
                }
                sq_dist = new_sq_dist;
                ++prune_index;
            }

            prune_index = std::min(prune_index, (int)(path.poses.size()-1));

        }
void Controller::FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                            const nav_msgs::Path& traj,
                            geometry_msgs::Twist& cmd_vel){
        geometry_msgs::PoseStamped robot_pose_1;
        GetTargetRobotPose(tf_listener, global_path.header.frame_id, robot_pose_1); 

        yaw = tf::getYaw(robot_pose.pose.orientation);

        //double diff_yaw = GetYawFromOrientation(traj.poses[0].pose.orientation)- GetYawFromOrientation(robot_pose.pose.orientation);
        double diff_yaw = atan2((traj.poses[1].pose.position.y-robot_pose.pose.position.y ),( traj.poses[1].pose.position.x-robot_pose.pose.position.x));
        
        double diff_distance = GetEuclideanDistance(robot_pose,traj.poses[1]);

        // set it from -PI t
        if(diff_yaw > M_PI){
            diff_yaw -= 2*M_PI;
        } else if(diff_yaw < -M_PI){
            diff_yaw += 2*M_PI;
        }

        printf("diff_yaw: %f\n",diff_yaw);
        printf("diff_distance: %f\n",diff_distance);

        double vx_global = max_speed*cos(diff_yaw)*p_value;//*diff_distance*p_value_;
        double vy_global = max_speed*sin(diff_yaw)*p_value;//*diff_distance*p_value_;
        std::cout<<"yaw_"<<yaw<<std::endl;
        std::cout<<"vx_gl  "<<vx_global<<"   vy_gl   "<<vy_global<<std::endl;
        
        cmd_vel.linear.x = vx_global * cos(yaw) + vy_global * sin(yaw);
        cmd_vel.linear.y = - vx_global * sin(yaw) + vy_global * cos(yaw);
        cmd_vel.angular.z = set_yaw_speed;

}
Controller::~Controller(){}
int main(int argc,char **argv)
{
  ros::init(argc, argv, "pid_position_follow");
  Controller controller;
  ros::spin();
    return 0;
}
