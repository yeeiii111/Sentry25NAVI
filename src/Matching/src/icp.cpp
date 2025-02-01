#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>//sac_ia
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //随机采样一致性去除
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <omp.h>
#include <mutex>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "preprocess.h"
shared_ptr<Preprocess> p_pre;
bool scan_start,scan_end;
int frame_count, frame_num;
std::string map_path,lid_topic;
double scan_start_time;
double scan_duration;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI());
typedef pcl::PointCloud<pcl::FPFHSignature33> pointfpfh;
void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    ROS_INFO("Cloud_received");
    scan_start = true;
    scan_start_time = omp_get_wtime();
    if(msg->header.stamp.toSec()<last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back");
        return;
    }
    if(frame_count > frame_num)
    {
        ROS_INFO("scan down");
        scan_duration = omp_get_wtime() - scan_start_time;
        ROS_INFO_STREAM("time cost:"<< scan_duration);
        scan_end = true;
        return;
    }
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg,ptr);
    *laserCloudWorld += *ptr;
    frame_count ++;
    ROS_INFO_STREAM("Frame num:"<<frame_count);
}
PointCloudXYZI::Ptr Icp_compute(PointCloudXYZI::Ptr source_cloud,PointCloudXYZI::Ptr target_cloud)
{
    pcl::IterativeClosestPoint<PointType,PointType> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.setEuclideanFitnessEpsilon(0.1);
    PointCloudXYZI::Ptr result(new PointCloudXYZI());
    icp.align(*result); 
    return result;
}
PointCloudXYZI::Ptr Ndt_compute(PointCloudXYZI::Ptr source_cloud,PointCloudXYZI::Ptr target_cloud)
{
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.01);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.1);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution(1.0);
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations(500);
    // 设置要配准的点云
    ndt.setInputCloud(source_cloud);
    //设置点云配准目标
    ndt.setInputTarget(target_cloud);
    PointCloudXYZI::Ptr result;
    ndt.align(*result); 
    return result;   
}

pointfpfh::Ptr compute_fpfh_feature(PointCloudXYZI::Ptr input_cloud, pcl::search::KdTree<PointType>::Ptr tree)
{
	//法向量
	pcl::PointCloud<pcl::Normal>::Ptr point_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointType, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
    est_normal.compute(*point_normal);//计算法向量
    ROS_INFO("Here3");
	//fpfh 估计
	pointfpfh::Ptr fpfh(new pointfpfh());
	pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(8); //指定4核计算
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	//est_fpfh.setRadiusSearch(0.08); 
	est_fpfh.compute(*fpfh);

	return fpfh;

}
PointCloudXYZI::Ptr fpfh_compute(PointCloudXYZI::Ptr source_cloud,PointCloudXYZI::Ptr target_cloud)
{
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>()); 
    pointfpfh::Ptr source_fpfh = compute_fpfh_feature(source_cloud,tree);
    pointfpfh::Ptr target_fpfh = compute_fpfh_feature(target_cloud,tree);
    ROS_INFO("Here2");
    pcl::io::savePCDFile("/home/wqx/RM/ForExperiment/src/Matching/PCD/source.pcd",*source_fpfh);
    pcl::io::savePCDFile("/home/wqx/RM/ForExperiment/src/Matching/PCD/target.pcd",*target_fpfh);
   
    pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(source_cloud);
	sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target_cloud);
    sac_ia.setTargetFeatures(target_fpfh);
    sac_ia.setNumberOfSamples(20);
    sac_ia.setCorrespondenceRandomness(10);
    sac_ia.setEuclideanFitnessEpsilon(0.001);
    sac_ia.setTransformationEpsilon(1e-10);
    sac_ia.setRANSACIterations(10);
    PointCloudXYZI::Ptr result(new PointCloudXYZI());
    sac_ia.align(*result);
    ROS_INFO("Here4");
    return result;
}
int main(int argc, char  **argv)
{
    p_pre.reset(new Preprocess());
    ros::init(argc,argv,"Matching");
    ros::NodeHandle nh("~");
    nh.param<std::string>("lid_topic", lid_topic, "/livox/lidar");
    nh.param<int>("frame_num", frame_num, 10);
    nh.param<std::string>("map_path", map_path, "/home/wqx/RM/catkin_livox/src/Point-LIO-master/PCD/scans.pcd");
    ros::Subscriber sub_pcl = nh.subscribe(lid_topic,200000,livox_pcl_cbk);
    PointCloudXYZI::Ptr Map_Cloud(new PointCloudXYZI());
    if(pcl::io::loadPCDFile<PointType>(map_path,*Map_Cloud) == -1)
    {
        ROS_ERROR("Load PCD file failed");
        return (-1);
    }
    while(ros::ok())
    {
        if(frame_count > frame_num)
        {
            ROS_INFO("Scan down");
            pcl::VoxelGrid<PointType> downSampled;
            downSampled.setInputCloud(laserCloudWorld);
            downSampled.setLeafSize(0.05,0.05,0.05);
            downSampled.filter(*laserCloudWorld);

            downSampled.setInputCloud(Map_Cloud);
            downSampled.setLeafSize(0.05,0.05,0.05);
            downSampled.filter(*Map_Cloud);

            pcl::StatisticalOutlierRemoval<PointType> OutlierRemoval;
            OutlierRemoval.setInputCloud(laserCloudWorld);
            OutlierRemoval.setMeanK(50);
            OutlierRemoval.setStddevMulThresh(3.0);
            OutlierRemoval.filter(*laserCloudWorld);

            PointCloudXYZI::Ptr rough_result(new PointCloudXYZI());
            PointCloudXYZI final_result;
            //粗配准
            ROS_INFO("Here0");
            rough_result = fpfh_compute(laserCloudWorld,Map_Cloud);
            if(rough_result->empty())
                ROS_INFO("rough_compute_failed");
            ROS_INFO("Here");
            final_result = *Icp_compute(rough_result,Map_Cloud);
            ROS_INFO("Here1");
            //可视化相关          
            pcl::visualization::PCLVisualizer viewer("demo");
            pcl::visualization::PointCloudColorHandlerCustom<PointType> source_color(laserCloudWorld, 255, 0, 0);  // 红色
            viewer.addPointCloud(laserCloudWorld, source_color, "source cloud");
            pcl::visualization::PointCloudColorHandlerCustom<PointType> target_color(Map_Cloud, 0, 255, 0);  // 绿色
            viewer.addPointCloud(Map_Cloud, target_color, "target cloud");
            pcl::visualization::PointCloudColorHandlerCustom<PointType> final_color(final_result.makeShared(), 0, 0, 255);  // 蓝色
            viewer.addPointCloud(final_result.makeShared(), final_color, "aligned cloud");
            // 启动可视化窗口
            while (!viewer.wasStopped())
            {
                viewer.spinOnce();
            }
        }
        if(~scan_end)
            ros::spinOnce();
    }

  

    return 0;
}


