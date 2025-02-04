#ifndef ROBOMASTER_UTILITY_H
#define ROBOMASTER_UTILITY_H
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "livox_ros_driver2/CustomMsg.h"

double GetEuclideanDistance(const geometry_msgs::PoseStamped & pose_1,
                            const geometry_msgs::PoseStamped & pose_2){
  return hypot(pose_1.pose.position.x-pose_2.pose.position.x,
               pose_1.pose.position.y-pose_2.pose.position.y);
}

bool GetTargetRobotPose(const std::shared_ptr<tf::TransformListener>& tf_listener,
                        const std::string& target_frame,
                        geometry_msgs::PoseStamped& robot_target_pose){
  tf::Stamped<tf::Pose> robot_pose_tf;
  robot_pose_tf.setIdentity();
  robot_pose_tf.frame_id_ = "base_link";
  robot_pose_tf.stamp_ = ros::Time::now() - ros::Duration(0.1);

  tf::Stamped<tf::Pose> robot_target_pose_tf;
  try{
    tf_listener->transformPose( target_frame, robot_pose_tf, robot_target_pose_tf);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Failed to transform robot pose: %s", ex.what());
    return false;
  }
  tf::poseStampedTFToMsg(robot_target_pose_tf, robot_target_pose);
  return true;
}

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