#include "segmentation/segmentation.hpp"
#include "segmentation/utility.hpp"
#include <pcl/common/transforms.h>

Obstacle_detector::Obstacle_detector():
    prior_map(new pcl::PointCloud<pcl::PointXYZ>),
    filtered_prior_map(new pcl::PointCloud<pcl::PointXYZ>),
    obstacle(new pcl::PointCloud<pcl::PointXYZ>),
    scan_sensor(new pcl::PointCloud<pcl::PointXYZ>),
    scan_map(new pcl::PointCloud<pcl::PointXYZ>){
    ros::NodeHandle nh("~");
    nh.param<double>("distance_threshold", distance_threshold, 0.2);
    nh.param<std::string>("map_path", map_path, "");
    nh.param<int>("freq", freq, 10);
    nh.param<std::vector<double>>("extrinsic_T", extrinT, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_R", extrinR, std::vector<double>());
    nh.param<double>("leaf_size", leaf_size, 0.05);
    nh.param<bool>("prior_map_pub_en", prior_map_pub_en, 0);
    Eigen::Vector3d translation(extrinT.data());
    Eigen::Matrix3d rotation;
    rotation << extrinR[0],extrinR[1],extrinR[2],
                extrinR[3],extrinR[4],extrinR[5],
                extrinR[6],extrinR[7],extrinR[8];
    T_sensor_baselink = Eigen::Affine3d::Identity();
    T_sensor_baselink.linear() = rotation;
    T_sensor_baselink.translation() = translation;

    scan_sub = nh.subscribe("/livox/lidar",5,&Obstacle_detector::Scan_Callback,this);
    obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle",10);
    prior_map_pub = nh.advertise<sensor_msgs::PointCloud2>("prior_map",10);
    tf_listener = std::make_shared<tf::TransformListener>();
    run_timer = nh.createTimer(ros::Duration(1.0/freq),&Obstacle_detector::timer,this);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(map_path,*prior_map) == -1)
    {
        ROS_ERROR("Load PCD file failed");
        ros::shutdown();
    }
    //方便KDtree有序存储
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(prior_map);
    downsample.setLeafSize(leaf_size,leaf_size,leaf_size);
    downsample.filter(*filtered_prior_map);
    kdtree.setInputCloud(filtered_prior_map);

}
void Obstacle_detector::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_map){
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
    GetTargetRobotPose(tf_listener,"map",robot_pose,last_scan_timestamp);

    Eigen::Vector3d position(robot_pose.pose.position.x,
                         robot_pose.pose.position.y,
                         robot_pose.pose.position.z);
    Eigen::Quaterniond orientation(robot_pose.pose.orientation.w,
                               robot_pose.pose.orientation.x,
                               robot_pose.pose.orientation.y,
                               robot_pose.pose.orientation.z);
    orientation.normalize();
    Eigen::Affine3d T_baselink_map = Eigen::Translation3d(position) * orientation.toRotationMatrix();
    Eigen::Affine3d T_sensor_map = T_sensor_baselink * T_baselink_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_scan(new pcl::PointCloud<pcl::PointXYZ>);
    {
        std::lock_guard<std::mutex> lock(scan_mutex);
        if(scan_sensor->size()!=0)
            *local_scan = *scan_sensor;
    }
    if(local_scan->size()!=0)
    {
        pcl::transformPointCloud(*local_scan, *scan_map, T_sensor_map);
        detect(scan_map);
        sensor_msgs::PointCloud2 obstacle_ros;
        pcl::toROSMsg(*obstacle,obstacle_ros);
        obstacle_ros.header.frame_id = "map";
        obstacle_ros.header.stamp = ros::Time::now();
        obstacle_pub.publish(obstacle_ros);
    }        
    if(prior_map_pub_en)
    {
        sensor_msgs::PointCloud2 prior_map_ros;
        pcl::toROSMsg(*filtered_prior_map,prior_map_ros);
        prior_map_ros.header.frame_id = "map";
        prior_map_pub.publish(prior_map_ros);
    }
}

void Obstacle_detector::Scan_Callback(const livox_ros_driver2::CustomMsg::ConstPtr &msg){
    std::lock_guard<std::mutex> lock(scan_mutex);
    livox_msg_handler(msg , scan_sensor);//自动覆盖scan_sensor
    last_scan_timestamp = scan_timestamp;  
    scan_timestamp = msg->header.stamp;
}
    
int main(int argc, char **argv)
{
    ros::init(argc,argv,"obstacle_detector");
    Obstacle_detector obstacle_detector;
    ros::spin();
    return 0;
}
