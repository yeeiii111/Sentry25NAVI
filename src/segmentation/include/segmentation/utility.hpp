#ifndef ROBOMASTER_UTILITY_H
#define ROBOMASTER_UTILITY_H
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "livox_ros_driver2/CustomMsg.h"
#ifdef   USE_GICP
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

double GetEuclideanDistance(const geometry_msgs::PoseStamped & pose_1,
                            const geometry_msgs::PoseStamped & pose_2){
  return hypot(pose_1.pose.position.x-pose_2.pose.position.x,
               pose_1.pose.position.y-pose_2.pose.position.y);
}

bool GetTargetPose(const std::shared_ptr<tf::TransformListener>& tf_listener,
                        const std::string& target_frame,
                        const std::string& source_frame,
                        geometry_msgs::PoseStamped& robot_target_pose,
                        ros::Time& timestamp){
  tf::Stamped<tf::Pose> robot_pose_tf;
  robot_pose_tf.setIdentity();
  robot_pose_tf.frame_id_ = source_frame;
  robot_pose_tf.stamp_ = timestamp;

  tf::Stamped<tf::Pose> robot_target_pose_tf;
  try{
    tf_listener->waitForTransform(target_frame, source_frame, timestamp, ros::Duration(0.2));
    tf_listener->transformPose( target_frame, robot_pose_tf, robot_target_pose_tf);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Failed to transform robot pose: %s", ex.what());
    return false;
  }
  tf::poseStampedTFToMsg(robot_target_pose_tf, robot_target_pose);
  return true;
}
#ifdef USE_GICP
void registration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, pcl::PointCloud<pcl::PointXYZ>::ConstPtr &target, double leaf_size)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source = small_gicp::voxelgrid_sampling_omp(*source, leaf_size , 6);
    auto mid_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> duration_mid = mid_time - start_time;

    std::cout << "downsample function took " << duration_mid.count() << " milliseconds" << std::endl;
    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;
    reg.setNumThreads(4);
    reg.setCorrespondenceRandomness(10);
    reg.setMaxCorrespondenceDistance(1.0);
    reg.setVoxelResolution(1.0);
    reg.setRegistrationType("VGICP");

    reg.setInputTarget(target);
    reg.setInputSource(filtered_source);

    auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    reg.align(*aligned);

    *source = *aligned;

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;

    std::cout << "registration function took " << duration.count() << " milliseconds" << std::endl;
}
#else
void livox_msg_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &pl_surf)
{
    int pl_size = msg->point_num;
    pcl::PointCloud<pcl::PointXYZ> pl_full;
    pl_surf->clear();
    pl_surf->reserve(pl_size);
    pl_full.resize(pl_size);
    for(uint i = 1; i < pl_size; i++)
    {
      pl_full[i].x = msg->points[i].x;
      pl_full[i].y = msg->points[i].y;
      pl_full[i].z = msg->points[i].z;
    
      if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
          || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
          || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
          && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > 0.01f))
      {
        pl_surf->push_back(pl_full[i]);
      }
    }

}
#endif
void standard_msg_handler(const sensor_msgs::PointCloud2ConstPtr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &pl_surf)
{
  int pl_size = msg->height * msg->width;
  pcl::PointCloud<pcl::PointXYZ> pl_full;
  pl_surf->clear();
  pl_surf->reserve(pl_size);
  pl_full.resize(pl_size);
  pcl::fromROSMsg(*msg, pl_full);
  for(uint i = 1; i < pl_size; i++)
  {
    if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
        || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
        || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > 0.01f)
        && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z < 50.0f))
    {
      pl_surf->push_back(pl_full[i]);
    }
  }
}
#endif