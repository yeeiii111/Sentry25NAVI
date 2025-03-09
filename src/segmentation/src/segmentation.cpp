#include "segmentation/segmentation.hpp"
#include "segmentation/utility.hpp"
#include <pcl/common/transforms.h>

Obstacle_detector::Obstacle_detector():
    prior_map (new pcl::PointCloud<pcl::PointXYZ>),
    filtered_prior_map (new pcl::PointCloud<pcl::PointXYZ>),
    obstacle (new pcl::PointCloud<pcl::PointXYZ>),
    scan_sensor (new pcl::PointCloud<pcl::PointXYZ>),
    scan_map (new pcl::PointCloud<pcl::PointXYZ>),
    last_scan_sensor(new pcl::PointCloud<pcl::PointXYZ>){

    ros::NodeHandle nh("~");
    nh.param<double>("distance_threshold", distance_threshold, 0.2);
    nh.param<std::string>("map_path", map_path, "");
    nh.param<int>("freq", freq, 10);
    nh.param<std::vector<double>>("extrinsic_T", extrinT, std::vector<double>());
    nh.param<std::vector<double>>("extrinsic_R", extrinR, std::vector<double>());
    nh.param<std::vector<double>>("IMU_extrinsic_T", IMU_extrinT, std::vector<double>());
    nh.param<std::vector<double>>("IMU_extrinsic_R", IMU_extrinR, std::vector<double>());
    nh.param<double>("leaf_size", leaf_size, 0.05);
    nh.param<bool>("prior_map_pub_en", prior_map_pub_en, 0);
    nh.param<bool>("use_livox_cloud",use_livox_cloud, 0);
    nh.param<double>("diverge_threshold",diverge_threshold,0.5);
    Eigen::Vector3d translation(extrinT.data());
    Eigen::Matrix3d rotation;
    rotation << extrinR[0],extrinR[1],extrinR[2],
                extrinR[3],extrinR[4],extrinR[5],
                extrinR[6],extrinR[7],extrinR[8];
    Eigen::Vector3d imu_translation(IMU_extrinT.data());
    Eigen::Matrix3d imu_rotation;
    imu_rotation << IMU_extrinR[0],IMU_extrinR[1],IMU_extrinR[2],
                    IMU_extrinR[3],IMU_extrinR[4],IMU_extrinR[5],
                    IMU_extrinR[6],IMU_extrinR[7],IMU_extrinR[8];
    if(use_livox_cloud == 1)
    {
        T_baselink_sensor = Eigen::Isometry3d::Identity();
        T_baselink_sensor.linear() = rotation;
        T_baselink_sensor.translation() = translation;        
    }
    else{
        T_baselink_sensor = Eigen::Isometry3d::Identity();
        T_baselink_sensor.linear() = imu_rotation;
        T_baselink_sensor.translation() = imu_translation;          
    }
    // //map frame绕z轴转-90度是odom frame
    // Eigen::AngleAxisd yaw(-M_PI/2,Eigen::Vector3d::UnitZ());
    // T_map_odom = Eigen::Affine3d::Identity();
    // T_map_odom.linear() = yaw.toRotationMatrix();
    //-------------------------------livox_cloud or standard cloud-------------------------------//
    if(use_livox_cloud == 1)
    {
        scan_sub = nh.subscribe("/livox/lidar",5,&Obstacle_detector::Livox_Scan_Callback,this);
    }
    else{
        scan_sub = nh.subscribe("/cloud_registered",5,&Obstacle_detector::Standard_Scan_Callback,this);
    }
    diverge_pub = nh.advertise<std_msgs::Bool>("/diverge",10);
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
    // pcl::PointCloud<pcl::PointXYZ> tem;
    downsample.filter(*filtered_prior_map);
    //filtered_prior_map = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(tem);
    kdtree.setInputCloud(filtered_prior_map);
    std::cout<<filtered_prior_map->size()<<std::endl;
}
void Obstacle_detector::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_map){
    auto start_time = std::chrono::high_resolution_clock::now();
    for (const auto& pt : scan_map->points){
        std::vector<int> indices(1);
        std::vector<float> sqr_distance(1);
        if(kdtree.nearestKSearch(pt, 1, indices, sqr_distance)>0){
            if (sqrt(sqr_distance[0]) > distance_threshold){
                obstacle->push_back(pt);
            }
        }
    }  
    double per = static_cast<double>(obstacle->size())/scan_map->size();
    if(per > diverge_threshold)
        diverge.data = true;
    else
        diverge.data = false;
    auto end_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    std::cout << "detect function took " << duration.count() << " milliseconds" << std::endl;    
    std::cout << "obstacle point percentage " << per*100<<"%" <<std::endl;
}
void Obstacle_detector::timer(const ros::TimerEvent &event){

    obstacle->clear();
    geometry_msgs::PoseStamped robot_pose;
    if(use_livox_cloud == 1){
        GetTargetPose(tf_listener,"map","base_link",robot_pose,last_scan_timestamp);
    }
    else{
        GetTargetPose(tf_listener,"map","odom",robot_pose,last_scan_timestamp);       
    }
    Eigen::Vector3d position(robot_pose.pose.position.x,
                         robot_pose.pose.position.y,
                         robot_pose.pose.position.z);
    Eigen::Quaterniond orientation(robot_pose.pose.orientation.w,
                               robot_pose.pose.orientation.x,
                               robot_pose.pose.orientation.y,
                               robot_pose.pose.orientation.z);
    orientation.normalize();
    //target 是 baselink--用原始点云 odom--用pointlio点云
    Eigen::Isometry3d T_map_target ;
    T_map_target.linear() = orientation.toRotationMatrix();
    T_map_target.translation() = position;
    //-------------------------------livox_cloud or standard cloud-------------------------------//
    Eigen::Affine3d T_map_sensor;
    T_map_sensor = T_map_target * T_baselink_sensor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_local (new pcl::PointCloud<pcl::PointXYZ>());
    {
        std::lock_guard<std::mutex> lock(scan_mutex);
        if(last_scan_sensor->size()!=0)
            *scan_local = *last_scan_sensor;
    }
    if(scan_local->size()!=0)
    {
        pcl::transformPointCloud(*scan_local, *scan_map, T_map_sensor);
        //registration(scan_map,filtered_prior_map,leaf_size);
        detect(scan_map);
        sensor_msgs::PointCloud2 obstacle_ros;
        pcl::toROSMsg(*obstacle,obstacle_ros);
        obstacle_ros.header.frame_id = "map";
        obstacle_ros.header.stamp = ros::Time::now();
        obstacle_pub.publish(obstacle_ros);

        diverge_pub.publish(diverge);
    }        
    if(prior_map_pub_en)
    {
        sensor_msgs::PointCloud2 prior_map_ros;
        pcl::toROSMsg(*filtered_prior_map,prior_map_ros);
        prior_map_ros.header.frame_id = "map";
        prior_map_pub.publish(prior_map_ros);
    }
}

void Obstacle_detector::Livox_Scan_Callback(const livox_ros_driver2::CustomMsg::ConstPtr &msg){
    std::lock_guard<std::mutex> lock(scan_mutex);
    *last_scan_sensor = *scan_sensor;    
    livox_msg_handler(msg , scan_sensor);//自动覆盖scan_sensor
    last_scan_timestamp = scan_timestamp;  
    scan_timestamp = msg->header.stamp;
}
void Obstacle_detector::Standard_Scan_Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(scan_mutex);
    *last_scan_sensor = *scan_sensor; 
    standard_msg_handler(msg, scan_sensor);
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
