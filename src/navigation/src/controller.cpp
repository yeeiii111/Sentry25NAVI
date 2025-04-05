#include"navigation/controller.hpp"

Controller::Controller()
{
    ros::NodeHandle nh("~");
    nh.param<double>("max_speed", max_speed, 1.0);
    nh.param<double>("straight_p_value", straight_p_value, 2);
    nh.param<double>("curve_p_value",curve_p_value,1);
    nh.param<double>("set_yaw_speed", set_yaw_speed, 0);
    nh.param<int>("plan_frequency", plan_freq, 30);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance, 0.2);
    nh.param<double>("prune_ahead_distance", prune_ahead_dist, 0.5);
    nh.param<std::string>("global_frame", global_frame, "map");
    nh.param<double>("wz_const", wz_const, 0.5);
    nh.param<double>("turn_tolerance",turn_tolerance,0.3);
    nh.param<bool>("debug_en", debug_en ,false);
    nh.param<int>("straight_foresee_index", straight_foresee_index, 1);
    nh.param<int>("curve_foresee_index", curve_foresee_index, 1);
    nh.param<double>("wz_p_value", wz_p_value, 1);
    local_path_pub = nh.advertise<nav_msgs::Path>("local_path",5);
    global_path_sub = nh.subscribe("/move_base/GlobalPlanner/plan",5,&Controller::GlobalPathCallback,this);
    //diverge_sub = nh.subscribe("/diverge",5,&Controller::DivergeCallback,this);
    match_sub = nh.subscribe("/match",5,&Controller::MatchCallback,this);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    tf_listener = std::make_shared<tf::TransformListener>();
    plan_timer = nh.createTimer(ros::Duration(1.0/plan_freq),&Controller::Plan,this);
    std::cout << "max_speed ="<< max_speed << std::endl;
    prune_index = 0;
}

void Controller::Plan(const ros::TimerEvent& event){

    if(plan){
        ROS_INFO("plan_start");
        auto start = ros::Time::now();
        geometry_msgs::PoseStamped robot_pose;
        GetTargetRobotPose(tf_listener, global_path.header.frame_id, robot_pose);
        double EuclideanDistance = GetEuclideanDistance(robot_pose,global_path.poses.back());
        if (EuclideanDistance <= goal_dist_tolerance
            || prune_index == global_path.poses.size() - 1){
            // plan = false;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.z = set_yaw_speed;
            cmd_vel_pub.publish(cmd_vel);
            std::cout << cmd_vel.linear.z << std::endl;
            ROS_INFO("Planning Success!");
            prune_index = 0;
            return;
        }
        else if((EuclideanDistance > turn_tolerance && abs(YawErrorCal(robot_pose,global_path.poses[prune_index+5])) > M_PI/2) || turn_state)
        {
            double error = YawErrorCal(robot_pose,global_path.poses[prune_index+5]);
            if(abs(error) < M_PI/4 ){
                turn_state = false;
                return;
            }
            turn_state = true;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.z = signum(error) * wz_const;
            cmd_vel_pub.publish(cmd_vel);
            ROS_INFO("Turnning attitude!");
            std::cout << "yaw error = " << error << std::endl;
            return;
        }
        
        FindNearstPose(robot_pose,global_path,prune_index, prune_ahead_dist);
        // std::cout<< "global_path size:" << global_path.poses.size() << std::endl;
        nav_msgs::Path prune_path, local_path;

        prune_path.header.frame_id = global_frame;
        prune_path.poses.push_back(robot_pose);

        int i = prune_index;
        while(i < global_path.poses.size() && i - prune_index < 20){
            prune_path.poses.push_back(global_path.poses[i]);
            i++;
        }
        //此处，原GenTraj函数有时会在local path中加入Nan点，导致后续错误，删除这些点有时会导致路经平滑失败，即localpath为空
        std::cout << "prune_path.size = " <<prune_path.poses.size() << std::endl;
        GenTraj(prune_path, local_path);
        std::cout << "local_path.size = " <<local_path.poses.size() << std::endl;
        local_path.header.frame_id = "map";
        local_path_pub.publish(local_path);

        geometry_msgs::Twist cmd_vel;
        //平滑成功判断
        if(local_path.poses.size() >=2)
            FollowTraj(robot_pose, local_path, cmd_vel);
        else
            FollowTraj(robot_pose, prune_path, cmd_vel);     
        cmd_vel_pub.publish(cmd_vel);
    }   
    else{
        geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            cmd_vel.linear.z = 0;   // bool success or not
            cmd_vel_pub.publish(cmd_vel);
            if(debug_en){
                // geometry_msgs::PoseStamped robot_pose;
                // GetTargetRobotPose(tf_listener, "map", robot_pose);
                // double tem = anglelimit(tf2::getYaw(robot_pose.pose.orientation)+M_PI_2);
                // std::cout << "attitude:"<<tem<<std::endl;
                // std::cout << "yaw:" <<tf2::getYaw(robot_pose.pose.orientation)<<std::endl;
            }
    }
    

}
double Controller::YawErrorCal(const geometry_msgs::PoseStamped& robot_pose,
                                const geometry_msgs::PoseStamped& path_pose){
    double robot_attitude = anglelimit(tf2::getYaw(robot_pose.pose.orientation) + M_PI_2);
    if(debug_en){
        // std::cout << "robot_attitude:"<<robot_attitude<<std::endl;
    }
    double dx = path_pose.pose.position.x - robot_pose.pose.position.x;
    double dy = path_pose.pose.position.y - robot_pose.pose.position.y;
    double path_attitude = atan2(dy, dx);
    if(std::isnan(path_attitude))ROS_ERROR("path_attitude NAN");

    //double path_attitude = atan2(path.poses.back().pose.position.y , path.poses.back().pose.position.x );
    return anglelimit(path_attitude - robot_attitude);
}
void Controller::GlobalPathCallback(const nav_msgs::PathConstPtr & msg){
  if (!msg->poses.empty()){
      global_path = *msg;
    if(diverge == false)
      plan = true;
  }
}
// void Controller::DivergetatusCallback(const std_msgs::BoolConstPtr &msg){
//     if(msg->data == 1){
//         diverge = true;
//         plan = false;
//     }
//     else {
//         diverge = false;
//     }
// }
void Controller::MatchCallback(const std_msgs::BoolConstPtr &msg){
    if(msg->data == 1){
        diverge = false;
    }
    else {
        diverge = true;
        plan = false;
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
            std::cout << "prune_index = " << prune_index << std::endl;
        }
void Controller::FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                            const nav_msgs::Path& traj,
                            geometry_msgs::Twist& cmd_vel){
        geometry_msgs::PoseStamped robot_pose_1;
        GetTargetRobotPose(tf_listener, global_path.header.frame_id, robot_pose_1); 

        yaw = tf::getYaw(robot_pose.pose.orientation);
        double diff_distance = GetEuclideanDistance(robot_pose,traj.poses[1]);
        if (diff_distance < 1e-3) { // 避免除以零
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.z = 0;
            return;
        }
        //double diff_yaw = GetYawFromOrientation(traj.poses[0].pose.orientation)- GetYawFromOrientation(robot_pose.pose.orientation);
        int index;
        //曲率计算,高曲率的地方前视距离远一些
        double curvature;
        if(3 < traj.poses.size())
        {
            curvature = computeCurvature(traj.poses[0],traj.poses[1],traj.poses[2]);
        }
        else curvature = 0;
        if(curvature > 1) 
        {
            p_value = curve_p_value;
            index = std::min(curve_foresee_index, static_cast<int>(traj.poses.size()));
        }
        else 
        {
            p_value = straight_p_value;
            index = std::min(straight_foresee_index, static_cast<int>(traj.poses.size()));
        }
        double diff_yaw = atan2((traj.poses[index].pose.position.y-robot_pose.pose.position.y ),( traj.poses[index].pose.position.x-robot_pose.pose.position.x));
        if(std::isnan(diff_yaw))
        {
            ROS_ERROR("diff_yaw NAN");
            std::cout << "DISTANCE TO NEXT TARGET"<<GetEuclideanDistance(robot_pose,traj.poses[index])<< std::endl;
        }
        if(debug_en)
            std::cout << "curvature = "<<curvature <<std::endl;
        diff_yaw = anglelimit(diff_yaw);
        // printf("diff_yaw: %f\n",diff_yaw);
        // printf("diff_distance: %f\n",diff_distance);

        double vx_global = cos(diff_yaw)*diff_distance*p_value;//*diff_distance*p_value_;
        double vy_global = sin(diff_yaw)*diff_distance*p_value;//*diff_distance*p_value_;
        double speed = sqrt(vx_global*vx_global + vy_global*vy_global);
        if (speed > max_speed) {
            double ratio = max_speed / speed;
            vx_global *= ratio;
            vy_global *= ratio;
        }


        double error = YawErrorCal(robot_pose,traj.poses[index]);
        double wz = wz_p_value * error;
        cmd_vel.angular.z = wz;
        //对旋转的补偿，匀速旋转是，实际走过的是一段圆弧
        double yaw_calibrated = anglelimit(yaw - wz / (plan_freq *2)) ;
        // std::cout<<"yaw_error  "<<error<<"   wz   "<< wz <<std::endl;
        // std::cout<<"yaw_clibrated" << yaw_calibrated <<std::endl;
        cmd_vel.linear.x = vx_global * cos(yaw_calibrated) + vy_global * sin(yaw_calibrated);
        cmd_vel.linear.y = - vx_global * sin(yaw_calibrated) + vy_global * cos(yaw_calibrated);
        if(std::isnan(cmd_vel.linear.x )) cmd_vel.linear.x = 0;
        if(std::isnan(cmd_vel.linear.y)) cmd_vel.linear.y= 0;
        cmd_vel.linear.z = 0;


}
Controller::~Controller(){}
int main(int argc,char **argv)
{
  ros::init(argc, argv, "pid_position_follow");
  Controller controller;
  ros::spin();
    return 0;
}
