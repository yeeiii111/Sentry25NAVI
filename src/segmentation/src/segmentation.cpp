#include "segmentation/segmentation.hpp"
#include "segmentation/utility.hpp"
#include <pcl/common/transforms.h>

Obstacle_detector::Obstacle_detector():
    prior_map(new pcl::PointCloud<pcl::PointXYZ>),
    obstacle(new pcl::PointCloud<pcl::PointXYZ>){
    ros::NodeHandle nh("~");
    nh.param<double>("distance_threshold", distance_threshold, 0.2);
    nh.param<std::string>("map_path", map_path, "");
    nh.param<std::vector<double>>("extrinsic_T", extrinT, std::vector<double>());
    scan_sub = nh.subscribe("/livox/lidar",5,&Obstacle_detector::Scan_Callback,this);
    obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle",10);
    tf_listener = std::make_shared<tf::TransformListener>();
    run_timer = nh.createTimer(ros::Duration(1.0/freq),&Obstacle_detector::timer,this);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(map_path,*prior_map) == -1)
    {
        ROS_ERROR("Load PCD file failed");
    }

}
void Obstacle_detector::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr &map,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_map){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(map);
    for (const auto& pt : scan_map->points){
        std::vector<int> indices(1);
        std::vector<float> sqr_distance(1);
        if(kdtree.nearestKSearch(pt, 1, indices, sqr_distance)>0){
            if (sqrt(sqr_distance[0]) > distance_threshold){
                obstacle->push_back(pt);
            }
        }
    }
                
}
void Obstacle_detector::timer(const ros::TimerEvent &event){

    obstacle->clear();
    geometry_msgs::PoseStamped robot_pose;
    GetTargetRobotPose(tf_listener,"map",robot_pose);

    Eigen::Vector3d position(robot_pose.pose.position.x,
                         robot_pose.pose.position.y,
                         robot_pose.pose.position.z);
    Eigen::Quaterniond orientation(robot_pose.pose.orientation.w,
                               robot_pose.pose.orientation.x,
                               robot_pose.pose.orientation.y,
                               robot_pose.pose.orientation.z);
    orientation.normalize();
    Eigen::Affine3d T_baselink_map = Eigen::Translation3d(position) * orientation;
    Eigen::Affine3d T_sensor_map = T_sensor_baselink * T_baselink_map;
    pcl::transformPointCloud(*scan_sensor, *scan_map, T_sensor_map);
    detect(prior_map, scan_map);
    sensor_msgs::PointCloud2 obstacle_ros;
    pcl::toROSMsg(*obstacle,obstacle_ros);
    obstacle_ros.header.frame_id = "map";
    obstacle_ros.header.stamp = ros::Time::now();
    obstacle_pub.publish(obstacle_ros);
}
void Obstacle_detector::Scan_Callback(const sensor_msgs::PointCloud2ConstPtr& msg){

    pcl::fromROSMsg(*msg, *scan_sensor);//自动覆盖scan_sensor
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"obstacle_detector");
    Obstacle_detector obstacle_detector;
    ros::spin();
    return 0;
}
