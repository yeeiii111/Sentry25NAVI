#include "segmentation/segmentation.hpp"
#include "segmentation/utility.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
Obstacle_detector::Obstacle_detector():
    prior_map (new pcl::PointCloud<pcl::PointXYZ>),
    hole (new pcl::PointCloud<pcl::PointXYZ>),
    filtered_prior_map (new pcl::PointCloud<pcl::PointXYZ>),
    obstacle (new pcl::PointCloud<pcl::PointXYZ>),
    obs (new pcl::PointCloud<pcl::PointXYZ>),
    scan_sensor (new pcl::PointCloud<pcl::PointXYZ>),
    scan_map (new pcl::PointCloud<pcl::PointXYZ>),
    last_scan_sensor(new pcl::PointCloud<pcl::PointXYZ>),
    cropped_scan(new pcl::PointCloud<pcl::PointXYZ>),
    cropped_map(new pcl::PointCloud<pcl::PointXYZ>),
    cropped_obs(new pcl::PointCloud<pcl::PointXYZ>),
    costmap_points(new pcl::PointCloud<pcl::PointXY>),
    previous_icp_result(Eigen::Isometry3d::Identity()){
        
    register_ = std::make_shared<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
    ros::NodeHandle nh("~");
    nh.param<double>("distance_threshold", distance_threshold, 0.2);
    nh.param<double>("costmap_distance_threshold", costmap_distance_threshold, 0.05);
    nh.param<std::string>("map_path", map_path, "");
    nh.param<std::string>("hole_path", hole_path, "");
    nh.param<int>("freq", freq, 10);
    nh.param<double>("leaf_size", leaf_size, 0.05);
    nh.param<float>("box_size", box_size, 1.5);
    nh.param<bool>("prior_map_pub_en", prior_map_pub_en, 0);
    nh.param<bool>("use_livox_cloud",use_livox_cloud, 0);
    nh.param<double>("diverge_threshold",diverge_threshold,0.5);
    nh.param<double>("high_match_threshold",high_match_threshold,0.2);
    nh.param<double>("lidar_height",lidar_height,0.145);
    nh.param<double>("lidar_roll",lidar_roll,-0.349);
    nh.param<double>("lidar_y",lidar_y,0.11);
    nh.param<bool>("debug_en",debug_en,0);
    nh.param<bool>("align_en",align_en,0);
    nh.param<double>("max_dist_sq",max_dist_sq,1);
    nh.param<double>("max_iterations",max_iterations,100);
    register_->reduction.num_threads = 4;
    register_->rejector.max_dist_sq = max_dist_sq;
    register_->criteria.rotation_eps = 1e-7;
    register_->optimizer.max_iterations = max_iterations;
    //-------------------------------livox_cloud or standard cloud-------------------------------//
    if(use_livox_cloud == 1)
    {
        // scan_sub = nh.subscribe("/livox/lidar",5,&Obstacle_detector::Livox_Scan_Callback,this);
    }
    else{
        scan_sub = nh.subscribe("/cloud_registered_body",5,&Obstacle_detector::Standard_Scan_Callback,this);
    }
    diverge_pub = nh.advertise<std_msgs::Bool>("/diverge",10);
    match_pub = nh.advertise<std_msgs::Bool>("/match",10);
    obstacle_pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle",10);
    prior_map_pub = nh.advertise<sensor_msgs::PointCloud2>("prior_map",10);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("aligned",10);
    body_frame_pub = nh.advertise<geometry_msgs::PoseStamped>("/body_pose",10);

    obs_sub = nh.subscribe("/ground_segmentation/obstacle_cloud",1,&Obstacle_detector::Obs_Callback, this);
    tf_listener = std::make_shared<tf::TransformListener>();
    run_timer = nh.createTimer(ros::Duration(1.0/freq),&Obstacle_detector::timer,this);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(map_path,*prior_map) == -1)
    {
        ROS_ERROR("Load PCD file failed");
        ros::shutdown();
    }
    // if(pcl::io::loadPCDFile<pcl::PointXYZ>(hole_path,*hole) == -1)
    // {
    //     ROS_ERROR("Load Hole file failed");
    //     ros::shutdown();
    // }
    //如果用扫描得到的点云当作目标，要先把点云转换到地图坐标系，而不是建图时pointlio的起点坐标系
    Eigen::Affine3d   T_map_vice_map;
    T_map_vice_map = Eigen::Isometry3d::Identity();
    Eigen::AngleAxis roll(lidar_roll,Eigen::Vector3d::UnitY());
    T_map_vice_map.linear() = roll.toRotationMatrix();
    T_map_vice_map.translation().z() = lidar_height;
    T_map_vice_map.translation().x() = lidar_y;
    pcl::transformPointCloud(*prior_map, *prior_map, T_map_vice_map);

    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(prior_map);
    downsample.setLeafSize(leaf_size,leaf_size,leaf_size);
    downsample.filter(*filtered_prior_map);
    kdtree.setInputCloud(filtered_prior_map);
    box_center.x() = 0; 
    box_center.y() = 0;
    box_center.z() = 0;
    cloud_crop(filtered_prior_map,box_center,box_size,cropped_map);

}
void Obstacle_detector::cloud_crop(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector3d pos, float box_size, pcl::PointCloud<pcl::PointXYZ>::Ptr &result){
    //crop_box很蛊，放弃
    // pcl::CropBox<pcl::PointXYZ> box;
    std::cout << "in cloud size " << cloud->size() << std::endl;
    // box.setInputCloud(cloud);  // 输入点云
    // box.setMin(Eigen::Vector4f(pos.x() - box_size, pos.y() - box_size, pos.z()-box_size, 1.0));  // 立方体最小角点
    // box.setMax(Eigen::Vector4f(pos.x() + box_size, pos.y() + box_size, pos.z()+box_size, 1.0));    // 立方体最大角点
    // box.setRotation(Eigen::Vector3f::Identity());
    // box.setTranslation(Eigen::Vector3f::Identity());
    // box.setMin(Eigen::Vector4f(-box_size,  -box_size, -box_size, 1.0));  // 立方体最小角点
    // box.setMax(Eigen::Vector4f(box_size,  box_size, box_size, 1.0));    // 立方体最大角点
    // box.filter(*result);
    float x_min = pos.x()-box_size;
    float x_max = pos.x() + box_size;
    float y_min = pos.y()-box_size;
    float y_max = pos.y() + box_size;
    float z_min = pos.z()-box_size;
    float z_max = pos.z() + 1.5;
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr cond(new pcl::ConditionAnd<pcl::PointXYZ>);
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, x_min)));
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, x_max)));
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, y_min)));
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, y_max)));   
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, z_min)));
    cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, z_max)));
    pcl::ConditionalRemoval<pcl::PointXYZ> cond_filter;
    cond_filter.setInputCloud(cloud);
    cond_filter.setCondition(cond);
    cond_filter.filter(*result);     
    std::cout << "cropped_cloud size " << result->size() << std::endl;

}
void Obstacle_detector::detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_map){
    auto start_time = std::chrono::high_resolution_clock::now();
    // pcl::PassThrough<pcl::PointXYZ> passTh;
    // passTh.setInputCloud(scan_map);                                                    // 输入原始点云
    // passTh.setFilterFieldName("z");                                                 // 直通滤波将过滤的维度，可以是pcl::PointXYZRGB中任意维度
    // passTh.setFilterLimits(-0.255, 2.2);                                               // 阈值范围
    // passTh.setNegative(false);                                                       // true不保留范围内的点，false保留范围内的点
    // passTh.filter(*scan_map); 
    std::cout << "detect_scan size " << scan_map->size() <<std::endl;
    for (const auto& pt : scan_map->points){
        std::vector<int> indices_1(1), indices_2{1};
        std::vector<float> sqr_distance_1(1) ,sqr_distance_2{1};
        //滤出动态障碍物
        if(kdtree.nearestKSearch(pt, 1, indices_1, sqr_distance_1)>0){
            if (sqrt(sqr_distance_1[0]) > distance_threshold){
                obstacle->push_back(pt);
            }
        }
    }  
    double per = static_cast<double>(obstacle->size())/scan_map->size();
    if(per > diverge_threshold)
        {
            match.data = false;
            high_match = false;
            std::chrono::duration<double, std::milli> diverge_duration = std::chrono::high_resolution_clock::now() - match_time;
            if (diverge_duration > std::chrono::milliseconds(3000)) diverge.data = true;
        }
    else
    {
        if(per <= high_match_threshold) high_match = true;
        else    high_match = false;
        match.data = true;
        match_time = std::chrono::high_resolution_clock::now();
        diverge.data = false;
    }
        
    auto end_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    if(debug_en)
    {
        std::cout << "detect function took " << duration.count() << " milliseconds" << std::endl;    
        std::cout << "obstacle point percentage " << per*100<<"%" <<std::endl;
    }

}
void Obstacle_detector::timer(const ros::TimerEvent &event){
    auto start_time = std::chrono::high_resolution_clock::now();
    obstacle->clear();
    geometry_msgs::PoseStamped init_pose;
    geometry_msgs::PoseStamped robot_pose;
    geometry_msgs::PoseStamped body_pose;
    if(use_livox_cloud == 1){
        GetTargetPose(tf_listener,"map","base_link",init_pose,last_scan_timestamp);
    }
    else{
        GetTargetPose(tf_listener,"map","body",init_pose,last_scan_timestamp);       
    }
    GetTargetPose(tf_listener,"map","base_link",robot_pose,last_scan_timestamp);
    GetTargetPose(tf_listener,"vice_map","body",body_pose,last_scan_timestamp);

    Eigen::Vector3d robot_position(
                        robot_pose.pose.position.x,
                        robot_pose.pose.position.y,
                        robot_pose.pose.position.z);
    Eigen::Vector3d position(init_pose.pose.position.x,
                         init_pose.pose.position.y,
                         init_pose.pose.position.z);
    Eigen::Quaterniond orientation(init_pose.pose.orientation.w,
                               init_pose.pose.orientation.x,
                               init_pose.pose.orientation.y,
                               init_pose.pose.orientation.z);
    orientation.normalize();
    //target 是 baselink--用原始点云 odom--用pointlio点云
    Eigen::Isometry3d T_map_target ;
    T_map_target.linear() = orientation.toRotationMatrix();
    T_map_target.translation() = position;

    Eigen::Affine3d T_map_sensor;
    T_map_sensor = T_map_target;//这里主要是换成了affine
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_local (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs_map (new pcl::PointCloud<pcl::PointXYZ>());
    {
        std::lock_guard<std::mutex> lock(scan_mutex);
        if(last_scan_sensor->size()!=0)
            *scan_local = *last_scan_sensor;
    }
    {
        std::lock_guard<std::mutex> lock(obs_mutex);
        pcl::transformPointCloud(*obs, *obs_map, T_map_sensor);
    }
    if(scan_local->size()!=0)
    {
        pcl::transformPointCloud(*scan_local, *scan_map, T_map_sensor);
        if((abs(box_center.x() - robot_position.x()) > box_size - 1) || (abs(box_center.y() - robot_position.y()) > box_size - 1) )
        {
            box_center = robot_position;
            cloud_crop(filtered_prior_map,box_center,box_size,cropped_map);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan, filtered_obs;
        filtered_scan = small_gicp::voxelgrid_sampling_omp(*scan_map,leaf_size);
        filtered_obs = small_gicp::voxelgrid_sampling_omp(*obs_map,leaf_size);
        cloud_crop(filtered_scan,box_center,box_size,cropped_scan);
        cloud_crop(filtered_obs,box_center,box_size,cropped_obs);

        if(cropped_scan->size() != 0 )
        {
            if(align_en)
            {
                target_cov = small_gicp::voxelgrid_sampling_omp<
                            pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointCovariance>>(*cropped_map ,leaf_size, 4);
                small_gicp::estimate_covariances_omp(*target_cov,2, 4);
                target_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
                            target_cov, small_gicp::KdTreeBuilderOMP(4));
                source_cov = small_gicp::voxelgrid_sampling_omp<
                            pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*cropped_scan, leaf_size , 4);
                small_gicp::estimate_covariances_omp(*source_cov, 2, 4);
                source_tree = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
                            source_cov, small_gicp::KdTreeBuilderOMP(4));
                small_gicp::RegistrationResult result;
                result = register_->align(*target_cov,*source_cov,*target_tree,previous_icp_result);
                Eigen::Affine3d T (result.T_target_source);
                pcl::transformPointCloud(*cropped_obs,*cropped_obs,T);
                pcl::transformPointCloud(*cropped_scan,*cropped_scan,T);
            }

            auto mid_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration_mid = mid_time - start_time;
            std::cout << "transform & downsample & registration took " << duration_mid.count() << " milliseconds" << std::endl;

            // if(result.error < 10)
                detect(cropped_scan);
            if(align_en)
            {
                sensor_msgs::PointCloud2 aligned_ros;
                pcl::toROSMsg(*cropped_scan,aligned_ros);
                aligned_ros.header.frame_id = "map";
                aligned_ros.header.stamp = ros::Time::now();
                aligned_pub.publish(aligned_ros);
            }

            sensor_msgs::PointCloud2 obstacle_ros;
            pcl::transformPointCloud(*obstacle, *obstacle, T_map_sensor.inverse());//转换回body坐标系
            pcl::toROSMsg(*obstacle,obstacle_ros);
            obstacle_ros.header.frame_id = "body";
            obstacle_ros.header.stamp = ros::Time::now();
            obstacle_pub.publish(obstacle_ros);

            diverge_pub.publish(diverge);
            match_pub.publish(match);
            //发布未漂移的里程计信息供重定位使用
            if(high_match)
            {
                body_frame_pub.publish(body_pose);
                // std::cout << body_pose << std::endl;
            }

        }
    }        
    if(prior_map_pub_en)
    {
        sensor_msgs::PointCloud2 prior_map_ros;
        pcl::toROSMsg(*cropped_map,prior_map_ros);
        prior_map_ros.header.frame_id = "map";
        prior_map_pub.publish(prior_map_ros);
    }
}

// void Obstacle_detector::Livox_Scan_Callback(const livox_ros_driver2::CustomMsg::ConstPtr &msg){
//     std::lock_guard<std::mutex> lock(scan_mutex);
//     *last_scan_sensor = *scan_sensor;    
//     livox_msg_handler(msg , scan_sensor);//自动覆盖scan_sensor
//     last_scan_timestamp = scan_timestamp;  
//     scan_timestamp = msg->header.stamp;
// }
void Obstacle_detector::grid2pointcloud(const nav_msgs::OccupancyGrid& costmap, pcl::PointCloud<pcl::PointXY>& cloud)
{
    for(int y = 0 ; y < costmap.info.height; y++)
        for(int x = 0; x < costmap.info.width; x++)
        {
            int index = y * costmap.info.width + x;
            std::cout << "cost:" << costmap.data[index]  << std::endl;
            if(costmap.data[index] > 99)//静态障碍层
                {            
                    double wx, wy;
                    Grid2world(x, y, costmap, wx, wy);
                    pcl::PointXY flatpoint = {wx,wy};
                    cloud.push_back(flatpoint);

                }
        }
}

void Obstacle_detector::Standard_Scan_Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(scan_mutex);
    *last_scan_sensor = *scan_sensor; 
    standard_msg_handler(msg, scan_sensor);
    last_scan_timestamp = scan_timestamp;
    scan_timestamp = msg->header.stamp;
}
void Obstacle_detector::Costmap_Callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    if(!msg->data.empty())
    costmap = *msg;
}
void Obstacle_detector::Obs_Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> tem;
    std::lock_guard<std::mutex> lock(obs_mutex);
    obs->clear();
    int size = msg->height* msg->width;
    obs->resize(size);
    pcl::fromROSMsg(*msg, tem); 
    *obs = tem;     
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"obstacle_detector");
    Obstacle_detector obstacle_detector;
    ros::spin();
    return 0;
}
