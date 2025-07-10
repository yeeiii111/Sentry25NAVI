#include "relocalization/relocalization.hpp"
#include <small_gicp/util/downsampling_omp.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ia_ransac.h>//sac_ia
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
//用的是原始点云
Relocalization::Relocalization():
    prior_map(new pcl::PointCloud<pcl::PointXYZ>()),
    prior_obs(new pcl::PointCloud<pcl::PointXYZ>()),
    filtered_prior_map(new pcl::PointCloud<pcl::PointXYZ>()),
    filtered_prior_obs(new pcl::PointCloud<pcl::PointXYZ>()),
    scan(new pcl::PointCloud<pcl::PointXYZ>()),
    obs(new pcl::PointCloud<pcl::PointXYZ>()),
    scan_odom(new pcl::PointCloud<pcl::PointXYZ>()),
    high_features_map(new pcl::PointCloud<pcl::PointXYZ>()),
    high_features_scan(new pcl::PointCloud<pcl::PointXYZ>()),
    aligned(new pcl::PointCloud<pcl::PointXYZ>()),
    source_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>()),
    target_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>()),
    point_normal(new pcl::PointCloud<pcl::Normal>()),
    previous_icp_result(Eigen::Isometry3d::Identity()),
    T_relocalize(Eigen::Isometry3d::Identity()){
    register_ = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
    tf_listener = std::make_unique<tf2_ros::TransformListener>(tf2_buffer);
    ros::NodeHandle nh("relocalization");
    nh.param<std::string>("pcd_path", pcd_path, "");
    nh.param<std::string>("obs_path", obs_path, "");
    nh.param<double>("leaf_size", leaf_size, 0.1);
    nh.param<int>("freq",freq,10);
    nh.param<int>("max_frame",max_frame,10);
    nh.param<double>("map_leaf_size",map_leaf_size,0.5);
    nh.param<double>("max_dist_sq",max_dist_sq,1);
    nh.param<double>("max_iterations",max_iterations,100);
    nh.param<int>("num_neighbors",num_neighbors,10);
    nh.param<int>("num_threads",num_threads,4);
    nh.param<bool>("debug_en", debug_en, false);
    nh.param<bool>("use_stl_cloud", use_stl_cloud, false);
    nh.param<double>("diverge_threshold", diverge_threshold, 10.0f);
    nh.param<double>("precise_diverge_threshold", precise_diverge_threshold, 5.0f);
    nh.param<double>("lidar_height",lidar_height,0.32);
    nh.param<double>("lidar_pitch",lidar_roll,0.349);

    // diverge.data = 0;
    register_->reduction.num_threads = num_threads;
    register_->rejector.max_dist_sq = max_dist_sq;
    register_->optimizer.max_iterations = max_iterations;
    obs_sub = nh.subscribe("/ground_segmentation/obstacle_cloud",5,&Relocalization::Obs_Callback,this);
    scan_sub = nh.subscribe("/livox/lidar_ros",5,&Relocalization::Standard_Scan_Callback,this);

    initial_pose_sub = nh.subscribe("/initialpose",5,&Relocalization::InitialPoseCallback,this);
    diverge_sub = nh.subscribe("/diverge",5,&Relocalization::Diverge_Callback,this);
    //match_sub = nh.subscribe("/match",5,&Relocalization::Match_Callback,this);
    odom_sub = nh.subscribe("/body_pose",5,&Relocalization::Body_Pose_Callback,this);
    target_pub = nh.advertise<sensor_msgs::PointCloud2>("target",5);
    source_pub = nh.advertise<sensor_msgs::PointCloud2>("source",5);
    align_pub = nh.advertise<sensor_msgs::PointCloud2>("aligned",5);
    localize_success_pub= nh.advertise<std_msgs::Bool>("/localize_success",30);
    localize_success = false;
    last_diverge_data = false;
    lost = false;
    y_dist = 0;
    previous_error = 10000;
    yaw_bias_cnt = 0;
    run_timer = nh.createTimer(ros::Duration(1.0/freq),&Relocalization::timer,this);

    if(pcl::io::loadPCDFile(pcd_path,*prior_map)==-1){
        ROS_ERROR("Load PCD file failed");
        ros::shutdown();
    }
    else{
        ROS_INFO("Load PCD file success");
    }
    if(pcl::io::loadPCDFile(obs_path,*prior_obs)==-1){
        ROS_ERROR("Load OBS file failed");
        ros::shutdown();
    }
    else{
        ROS_INFO("Load OBS file success");
    }
    target_cov = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointCovariance>>(*prior_map ,map_leaf_size, num_threads);
    filtered_prior_map = small_gicp::voxelgrid_sampling_omp(*prior_map ,map_leaf_size, 8);
    filtered_prior_obs = small_gicp::voxelgrid_sampling_omp(*prior_obs ,map_leaf_size, 8);
    // if(use_stl_cloud == false){
    //     T_map_vice_map = Eigen::Isometry3d::Identity();
    //     Eigen::AngleAxis roll(lidar_roll,Eigen::Vector3d::UnitY());
    //     T_map_vice_map.linear() = roll.toRotationMatrix();
    //     T_map_vice_map.translation().z() = lidar_height;
    //     pcl::transformPointCloud(*filtered_prior_map, *filtered_prior_map, T_map_vice_map);
    // }    
    // //保留高处点云做粗配准
    // pcl::PassThrough<pcl::PointXYZ> passTh;
    // passTh.setInputCloud(filtered_prior_map);                                                    // 输入原始点云
    // passTh.setFilterFieldName("z");                                                 // 直通滤波将过滤的维度，可以是pcl::PointXYZRGB中任意维度
    // passTh.setFilterLimits(-1, 3);                                               // 阈值范围
    // passTh.setNegative(false);                                                       // true不保留范围内的点，false保留范围内的点
    // passTh.filter(*filtered_prior_map); 
    // if(use_stl_cloud == false){
    //     //pcl::transformPointCloud(*high_features_map, *high_features_map, T_map_vice_map.inverse());
    //     pcl::transformPointCloud(*filtered_prior_map, *filtered_prior_map, T_map_vice_map.inverse());
    // }

    small_gicp::estimate_covariances_omp(*target_cov,num_neighbors, num_threads);
    target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
        target_cov, small_gicp::KdTreeBuilderOMP(num_threads)
    );

    T_odom_lidar = Eigen::Isometry3d::Identity();
    T_odom_lidar.translation().z() = 0.0f;//0.36f pointlio生成的点云在imu坐标系下，直接扫描的点云在lidar坐标系下
    ROS_INFO("Initialization done");
}
void Relocalization::timer(const ros::TimerEvent& event)
{
    T_vice_map_odom.header.frame_id = "vice_map";
    T_vice_map_odom.child_frame_id = "odom";
    if(!obs_queue.empty() && !scan_queue.empty())
    // if( !scan_queue.empty())
    {
        if(localize_success == false || diverge.data == true )
        {
            {
                std::lock_guard<std::mutex> lock(obs_mutex);
                obs->clear();
                while(!obs_queue.empty())
                {
                    *obs += obs_queue.front();
                    obs_queue.pop();
                }
            }
            {
                std::lock_guard<std::mutex> lock(scan_mutex);
                scan->clear();
                while(!scan_queue.empty())
                {
                    *scan += scan_queue.front();
                    scan_queue.pop();
                }
                Eigen::Affine3d tem(T_odom_lidar);
                pcl::transformPointCloud(*scan,*scan_odom,tem);
            }

            registration(scan_odom, obs, leaf_size, diverge_threshold); 

            Eigen::Quaterniond q(T_map_scan.linear());
            Eigen::Vector3d translation =  T_map_scan.translation();  

            T_vice_map_odom.transform.rotation.x = q.x();
            T_vice_map_odom.transform.rotation.y = q.y();
            T_vice_map_odom.transform.rotation.z = q.z();
            T_vice_map_odom.transform.rotation.w = q.w();  

            T_vice_map_odom.transform.translation.x = translation.x();
            T_vice_map_odom.transform.translation.y = translation.y();
            T_vice_map_odom.transform.translation.z = translation.z(); 
            std::cout<< "qx = " << q.x() << std::endl;
            std::cout<< "qy = " << q.y() << std::endl;
            std::cout<< "qz = " << q.z() << std::endl;
            std::cout<< "qw = " << q.w() << std::endl;

            std::cout<<"x = "<< translation.x() << std::endl;
            std::cout<<"y = "<< translation.y() << std::endl;
            std::cout<<"z = "<< translation.z() << std::endl;
            broadcaster.sendTransform(T_vice_map_odom);
        }
        std_msgs::Bool tem;
        tem.data=localize_success;
        localize_success_pub.publish(tem);  
        // if(match.data == true && localize_success == true)
        // {
        //     geometry_msgs::TransformStamped robot_pose;
        //     bool success;
        //     try{
        //         robot_pose = tf2_buffer.lookupTransform("vice_map","body",ros::Time(0));
        //         success = true;
        //     }catch(tf2::TransformException &e){
        //         ROS_ERROR("transform failed %s", e.what());
        //         success = false ;
        //     }
        //     if(success)
        //     {
        //         y_dist = robot_pose.transform.translation.y;
        //         // 提取平移部分并赋值给 Isometry3d
        //         Eigen::Vector3d translation(
        //             robot_pose.transform.translation.x,
        //             robot_pose.transform.translation.y,
        //             robot_pose.transform.translation.z
        //         );
        //         previous_icp_result.translation() = translation;
        //         T_relocalize.translation() = translation;
        //         // 提取四元数并转换为旋转矩阵，赋值给 Isometry3d
        //         Eigen::Quaterniond quat(
        //             robot_pose.transform.rotation.w,
        //             robot_pose.transform.rotation.x,
        //             robot_pose.transform.rotation.y,
        //             robot_pose.transform.rotation.z
        //         );
        //         previous_icp_result.linear() = quat.toRotationMatrix();
        //         T_relocalize.linear() = quat.toRotationMatrix();
        //         // std::cout << translation.x() << std::endl;
        //         // std::cout << translation.y() << std::endl;
        //         // std::cout << translation.z() << std::endl;  
        //     }         
        // }
    }
    //debug
    if(debug_en)
    {
        sensor_msgs::PointCloud2 msg_1;
        pcl::toROSMsg(*filtered_prior_map, msg_1);
        msg_1.header.frame_id = "vice_map";  
        target_pub.publish(msg_1); 

        sensor_msgs::PointCloud2 msg_2;
        pcl::toROSMsg(*scan_odom, msg_2);
        msg_2.header.frame_id = "vice_map";  
        source_pub.publish(msg_2); 

        sensor_msgs::PointCloud2 msg_3;
        pcl::toROSMsg(*aligned, msg_3);
        msg_3.header.frame_id = "vice_map";  
        align_pub.publish(msg_3);       
    }

}
void Relocalization::registration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &obs, double leaf_size, double threshold)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_odom(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_obs(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_;
    sor_.setInputCloud(obs);
    sor_.setMeanK(20);
    sor_.setStddevMulThresh(0.02);
    sor_.filter(*filtered_obs);
    filtered_obs = small_gicp::voxelgrid_sampling_omp(*filtered_obs ,leaf_size, 8);
    filtered_scan_odom = small_gicp::voxelgrid_sampling_omp(*source ,leaf_size, 8);
    source_cov = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*source, leaf_size , num_threads);
    
    // if(use_stl_cloud == false){
    //     pcl::transformPointCloud(*filtered_scan_odom, *filtered_scan_odom, T_map_vice_map);
    // }   

    // pcl::PassThrough<pcl::PointXYZ> passTh;
    // passTh.setInputCloud(filtered_scan_odom);                                                    // 输入原始点云
    // passTh.setFilterFieldName("z");                                                 // 直通滤波将过滤的维度，可以是pcl::PointXYZRGB中任意维度
    // passTh.setFilterLimits(-1, 3);                                               // 阈值范围
    // passTh.setNegative(false);                                                       // true不保留范围内的点，false保留范围内的点
    // passTh.filter(*filtered_scan_odom); 
    // passTh.setInputCloud(filtered_scan_odom);                                                    // 输入原始点云
    // passTh.filter(*filtered_scan_odom);
    // if(use_stl_cloud == false){
    //     pcl::transformPointCloud(*filtered_scan_odom, *filtered_scan_odom, T_map_vice_map.inverse());
    //     //pcl::transformPointCloud(*high_features_scan, *high_features_scan, T_map_vice_map.inverse());
    // }  

    small_gicp::estimate_covariances_omp(*source_cov, num_neighbors, num_threads);
    source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
                source_cov, small_gicp::KdTreeBuilderOMP(num_threads));
    auto mid_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration_mid = mid_time - start_time;
    std::cout << "downsample function took " << duration_mid.count() << " milliseconds" << std::endl;
    small_gicp::RegistrationResult result;
    //-------------------针对联盟赛注释掉了粗匹配部分------------------------
    if(lost)
    {
        sac_ia_compute(filtered_obs,filtered_prior_obs);
        // if(y_dist < 5.25 && fpfh_result.translation().y() > 5.25){
        //     Eigen::AngleAxis tem(M_PI,Eigen::Vector3d::UnitZ());
        //     fpfh_result.linear() = tem * fpfh_result.linear().eval();
        //     fpfh_result.translation().x() = -fpfh_result.translation().x();
        //     fpfh_result.translation().y() = 10.5-fpfh_result.translation().y();
        //     ROS_INFO("-------------HALFCOURT----------------------");
        // }
        result = register_->align(*target_cov,*source_cov,*target_tree,fpfh_result);
        previous_error = 100000.0f;
        lost = false;
    }
    else
        result = register_->align(*target_cov,*source_cov,*target_tree,previous_icp_result);
    T_map_scan = result.T_target_source;
    previous_icp_result = result.T_target_source;
    if(result.error < threshold){
        localize_success  = true;
        previous_error = 10000;
        yaw_bias_cnt = 0;
        lost = false;
    }
    else {
        localize_success = false;
        if(result.error >= previous_error || (previous_error - result.error < 1))//无法收敛时换一个先验位姿
        {
            previous_error = 100000.0f;
            // Eigen::Isometry3d T_rotation = Eigen::Isometry3d::Identity();
            double yaw_bias;
            if(yaw_bias_cnt % 2 == 0) yaw_bias = (yaw_bias_cnt/2 + 1) * 0.349f;//每次旋转20度
            else yaw_bias = -(yaw_bias_cnt/2 + 1) * 0.349f;
            std::cout << "-------------------yaw_bias:-------------------" << yaw_bias <<std::endl;
            double angle_x = -20.0 * M_PI / 180.0;  // 转换为弧度
            Eigen::AngleAxisd rotate_x(angle_x, Eigen::Vector3d::UnitY());
            Eigen::Vector3d new_axis = rotate_x * Eigen::Vector3d::UnitZ();
            Eigen::AngleAxis yaw(yaw_bias,new_axis);
            T_relocalize.rotate(yaw.toRotationMatrix());
            previous_icp_result = T_relocalize;
            yaw_bias_cnt ++; 
            // lost 一直为false就不会进入粗配准
            // lost = true;
        }
        else previous_error = result.error;
    }
    if(debug_en){
        Eigen::Affine3d tem(T_map_scan);
        pcl::transformPointCloud(*filtered_scan_odom,*aligned,tem);   
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    std::cout<<"error = "<<result.error<<std::endl;
    std::cout << "registration function took " << duration.count() << " milliseconds" << std::endl;
    std::cout << "registration result: " << localize_success << std::endl;

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
        Eigen::Isometry3d T_vice_map_odom = T_map_baselink_estimate * T_baselink_odom;//和全局变量区分
        previous_icp_result = T_vice_map_odom;
        previous_error = 100000.0f;
        localize_success = false;
        lost = false;
        // std::cout << "---------------------------------------------------------------"<< std::endl;
        // std::cout << "T_map_baselink_estimate linear: "<< std::endl;
        // std::cout << T_map_baselink_estimate.linear() << std::endl;
        // std::cout << "T_map_baselink_estimate translation: "<< std::endl;
        // std::cout<< T_map_baselink_estimate.translation() << std::endl;
        // std::cout << "T_baselink_odom linear: "<< std::endl;
        // std::cout<< T_baselink_odom.linear() << std::endl;
        // std::cout << "T_baselink_odom translation: "<< std::endl;
        // std::cout<< T_baselink_odom.translation() << std::endl;
        // std::cout << "--------------------------------------------------------------"<< std::endl;
   
            }catch(tf2::TransformException & ex){
        ROS_WARN("Could not transform initial pose");
    }

}

void Relocalization::Obs_Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{  
    pcl::PointCloud<pcl::PointXYZ> tem;
    pcl::fromROSMsg(*msg, tem); 
    {
        std::lock_guard<std::mutex> lock(obs_mutex);
        obs_queue.push(tem);
        if(obs_queue.size() > max_frame)
            obs_queue.pop();
    }
}
void Relocalization::Standard_Scan_Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{ 
    pcl::PointCloud<pcl::PointXYZ> tem;
    // std::lock_guard<std::mutex> lock(scan_mutex);
    // scan->clear();
    // int size = msg->height* msg->width;
    // scan->resize(size);
    pcl::fromROSMsg(*msg, tem); 
    {
        std::lock_guard<std::mutex> lock(scan_mutex);
        scan_queue.push(tem);
        if(scan_queue.size() > max_frame)
            scan_queue.pop();
    }
   // *scan = tem;     
}
void Relocalization::Diverge_Callback(const std_msgs::Bool::ConstPtr &msg)
{
    last_diverge_data = diverge.data;
    diverge.data = msg->data;
    if(diverge.data) localize_success = false;
    std_msgs::Bool tem;
    tem.data=localize_success;
    localize_success_pub.publish(tem);  
}
void Relocalization::Match_Callback(const std_msgs::Bool::ConstPtr &msg)
{
    match.data = msg->data;
}
void Relocalization::Body_Pose_Callback(const geometry_msgs::PoseStamped &msg)
{
    if(localize_success)
    {
        Eigen::Vector3d translation(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z    
        );
        previous_icp_result.translation() = translation;
        Eigen::Quaterniond quat(
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        );
        previous_icp_result.linear() = quat.toRotationMatrix();
    }

}

void Relocalization::compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
                                        pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree,
                                        pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh){
	//法向量
    //此处 est_normal est_fpfh如果创建为局部变量，在函数退出时有bug会导致指针重复释放（eigen和pcl本身的问题）
    point_normal->clear();
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
    est_normal.setRadiusSearch(0.5);
	// est_normal.setKSearch(10);
    est_normal.compute(*point_normal);//计算法向量
	//fpfh 估计
	est_fpfh.setNumberOfThreads(4);
    est_fpfh.setRadiusSearch(1);
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	// est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);
    std::cout <<"fpfh size: "<< fpfh->size() << std::endl;
    if(fpfh->empty()){
        ROS_ERROR("FPFH feature search failed!");
    }    
}
pcl::PointCloud<pcl::PointXYZ> Relocalization::sac_ia_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                                                pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>()); 
    // pcl::PointCloud<pcl::PointXYZ>::Ptr source_iss(new pcl::PointCloud<pcl::PointXYZ>());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr target_iss(new pcl::PointCloud<pcl::PointXYZ>());
    // *source_iss = ISS_compute(source_cloud,tree);
    // *target_iss = ISS_compute(target_cloud,tree);
    compute_fpfh_feature(source_cloud,tree,source_fpfh);
    compute_fpfh_feature(target_cloud,tree,target_fpfh);

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source_cloud);
	sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target_cloud);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setNumberOfSamples(30);
    sac_ia.setMinSampleDistance(1);
    sac_ia.setCorrespondenceRandomness(20);
    sac_ia.setEuclideanFitnessEpsilon(0.00001);
    sac_ia.setTransformationEpsilon(1e-8);
    sac_ia.setRANSACIterations(500);
    sac_ia.setMaximumIterations(2000);
    
    pcl::PointCloud<pcl::PointXYZ> result;
    sac_ia.align(result);
    if (sac_ia.hasConverged()) {
        fpfh_result = sac_ia.getFinalTransformation().cast<double>();
    } else {
        ROS_ERROR("FPFH-based RANSAC failed to converge!");
        fpfh_result = Eigen::Isometry3d::Identity(); // 使用默认值
    }
    if(debug_en)
    {
        std::cout << "-------------approximate transform-----------------------"<< std::endl;
        std::cout << fpfh_result.linear() << std::endl;
        std::cout << fpfh_result.translation() << std::endl;
        std::cout << "-------------approximate transform-----------------------"<< std::endl;     

        // sensor_msgs::PointCloud2 msg_1;
        // pcl::toROSMsg(*source_iss, msg_1);
        // msg_1.header.frame_id = "map"; 

        // sensor_msgs::PointCloud2 msg_2;
        // pcl::toROSMsg(*target_iss, msg_2);
        // msg_2.header.frame_id = "map";   
  
    }
    auto mid_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration_mid = mid_time - start_time;
    std::cout << "sac_ia compute function took " << duration_mid.count() << " milliseconds" << std::endl;

    return result;
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"relocalization");
    Relocalization relocalization;
    ros::spin();
    return 0;
}