#include "relocalization/relocalization.hpp"
#include <small_gicp/util/downsampling_omp.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
Relocalization::Relocalization():
    prior_map(new pcl::PointCloud<pcl::PointXYZ>()),
    filtered_prior_map(new pcl::PointCloud<pcl::PointXYZ>()),
    scan(new pcl::PointCloud<pcl::PointXYZ>()),
    scan_odom(new pcl::PointCloud<pcl::PointXYZ>()),
    aligned(new pcl::PointCloud<pcl::PointXYZ>()),
    previous_icp_result(Eigen::Isometry3d::Identity()){
    register_ = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
    tf_listener = std::make_unique<tf2_ros::TransformListener>(tf2_buffer);
    ros::NodeHandle nh("relocalization");
    nh.param<std::string>("pcd_path", pcd_path, "");
    nh.param<double>("leaf_size", leaf_size, 0.1);
    nh.param<int>("freq",freq,10);
    nh.param<double>("map_leaf_size",map_leaf_size,0.5);
    nh.param<double>("max_dist_sq",max_dist_sq,1);
    nh.param<int>("num_neighbors",num_neighbors,10);
    nh.param<int>("num_threads",num_threads,4);
    register_->reduction.num_threads = num_threads;
    register_->rejector.max_dist_sq = max_dist_sq;

    scan_sub = nh.subscribe("/livox/lidar_ros",5,&Relocalization::Standard_Scan_Callback,this);
    initial_pose_sub = nh.subscribe("/initialpose",5,&Relocalization::InitialPoseCallback,this);
    target_pub = nh.advertise<sensor_msgs::PointCloud2>("target",5);
    source_pub = nh.advertise<sensor_msgs::PointCloud2>("source",5);
    align_pub = nh.advertise<sensor_msgs::PointCloud2>("aligned",5);

    run_timer = nh.createTimer(ros::Duration(1.0/freq),&Relocalization::timer,this);

    if(pcl::io::loadPCDFile(pcd_path,*prior_map)==-1){
        ROS_ERROR("Load PCD file failed");
        ros::shutdown();
    }
    //去掉点云中地面以下扫描不到的点
    // pcl::PassThrough<pcl::PointXYZ> passTh;
    // passTh.setInputCloud(prior_map);                                                    // 输入原始点云
    // passTh.setFilterFieldName("z");                                                 // 直通滤波将过滤的维度，可以是pcl::PointXYZRGB中任意维度
    // passTh.setFilterLimits(0.0, 100);                                               // 阈值范围
    // passTh.setNegative(false);                                                       // true不保留范围内的点，false保留范围内的点
    // passTh.filter(*prior_map); 

    target_cov = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointCovariance>>(*prior_map ,map_leaf_size, num_threads);
    filtered_prior_map = small_gicp::voxelgrid_sampling_omp(*prior_map ,map_leaf_size, 8);
    small_gicp::estimate_covariances_omp(*target_cov,num_neighbors, num_threads);
    target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_cov, small_gicp::KdTreeBuilderOMP(num_threads)
    );
    //stl直接得到的map frame绕z轴转-90度是odom frame
    Eigen::AngleAxisd yaw(-M_PI/2,Eigen::Vector3d::UnitZ());
    T_odom90_odom = Eigen::Isometry3d::Identity();
    T_odom90_odom.linear() = yaw.toRotationMatrix();
    T_odom_lidar = Eigen::Isometry3d::Identity();
    T_odom_lidar.translation().z() = 0.11f;//0.36f pointlio生成的点云在imu坐标系下，直接扫描的点云在lidar坐标系下
    relocalization_done = false;
}
void Relocalization::timer(const ros::TimerEvent& event)
{
    T_map_odom.header.frame_id = "map";
    T_map_odom.child_frame_id = "odom";
    if(!relocalization_done == 1 && !scan->empty() )
    {
        {
            std::lock_guard<std::mutex> lock(scan_mutex);
            //Eigen::Affine3d tem(T_odom90_odom * T_odom_lidar);
            Eigen::Affine3d tem(T_odom_lidar);
            pcl::transformPointCloud(*scan,*scan_odom,tem);
            // 改用pointlio得到的点云后scan就是scan_odom
            //*scan_odom = *scan;
        }
        registration(scan_odom, leaf_size);  
        Eigen::Quaterniond q(T_map_scan.linear());
        Eigen::Vector3d translation =  T_map_scan.translation();  

        T_map_odom.transform.rotation.x = q.x();
        T_map_odom.transform.rotation.y = q.y();
        T_map_odom.transform.rotation.z = q.z();
        T_map_odom.transform.rotation.w = q.w();  

        T_map_odom.transform.translation.x = translation.x();
        T_map_odom.transform.translation.y = translation.y();
        T_map_odom.transform.translation.z = translation.z(); 
        std::cout<<"x = "<< translation.x() << std::endl;
        std::cout<<"y = "<< translation.y() << std::endl;
        std::cout<<"z = "<< translation.z() << std::endl;
        broadcaster.sendTransform(T_map_odom);
    }
    //debug
    if(relocalization_done == 1)
    {
        sensor_msgs::PointCloud2 msg_1;
        pcl::toROSMsg(*filtered_prior_map, msg_1);
        msg_1.header.frame_id = "map";  
        target_pub.publish(msg_1); 

        sensor_msgs::PointCloud2 msg_2;
        pcl::toROSMsg(*scan_odom, msg_2);
        msg_2.header.frame_id = "map";  
        source_pub.publish(msg_2); 

        sensor_msgs::PointCloud2 msg_3;
        pcl::toROSMsg(*aligned, msg_3);
        msg_3.header.frame_id = "map";  
        align_pub.publish(msg_3);       
    }

}
void Relocalization::registration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, double leaf_size)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    source_cov = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*source, leaf_size , num_threads);
    small_gicp::estimate_covariances_omp(*source_cov, num_neighbors, num_threads);
    // Create KdTree for source.
    source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_cov, small_gicp::KdTreeBuilderOMP(num_threads));

    auto mid_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration_mid = mid_time - start_time;

    std::cout << "downsample function took " << duration_mid.count() << " milliseconds" << std::endl;
    // small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;
    // reg.setNumThreads(12);
    // reg.setCorrespondenceRandomness(20);
    // reg.setMaxCorrespondenceDistance(1);
    // reg.setVoxelResolution(0.25);
    // reg.setRegistrationType("GICP");
    // reg.setEuclideanFitnessEpsilon(1e-6);
    // reg.setInputTarget(target);
    // reg.setInputSource(filtered_source);
    // reg.align(*aligned);
    // T_map_scan = reg.getRegistrationResult().T_target_source; 
    // *source = *aligned;
    auto result = register_->align(*target_cov,*source_cov,*target_tree,previous_icp_result);
    T_map_scan = result.T_target_source;
    pcl::transformPointCloud(*source,*aligned,T_map_scan);
    //T_map_scan = T_map_scan * T_odom90_odom;
    previous_icp_result = result.T_target_source;
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    std::cout<<"error = "<<result.error<<std::endl;
    std::cout << "registration function took " << duration.count() << " milliseconds" << std::endl;
    relocalization_done = 1;
}
void Relocalization::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    Eigen::Isometry3d T_map_baselink_estimate = Eigen::Isometry3d::Identity();
    T_map_baselink_estimate.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, 
                                             msg->pose.pose.position.z;
    T_map_baselink_estimate.linear() << Eigen::Quaternion(
                                            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                            msg->pose.pose.orientation.y ,msg->pose.pose.orientation.z).toRotationMatrix();
    try{
        auto transform = tf2_buffer.lookupTransform("base_link","odom",ros::Time(0));
        Eigen::Isometry3d T_baselink_odom = tf2::transformToEigen(transform.transform);
        Eigen::Isometry3d T_map_odom = T_map_baselink_estimate * T_baselink_odom;//和全局变量区分

        previous_icp_result = T_map_odom;
        relocalization_done = false;
    }catch(tf2::TransformException & ex){
        ROS_WARN("Could not transform initial pose");
    }

}

void Relocalization::Standard_Scan_Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if(!relocalization_done == 1)
    {   
        pcl::PointCloud<pcl::PointXYZ> tem;
        std::lock_guard<std::mutex> lock(scan_mutex);
        scan->clear();
        int size = msg->height* msg->width;
        scan->resize(size);
        pcl::fromROSMsg(*msg, tem); 
        *scan += tem; 
        // pointcloud_count ++;      
    }

}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"relocalization");
    Relocalization relocalization;
    ros::spin();
    return 0;
}