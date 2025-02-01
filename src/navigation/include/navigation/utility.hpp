#ifndef ROBOMASTER_UTILITY_H
#define ROBOMASTER_UTILITY_H
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

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
  robot_pose_tf.stamp_ = ros::Time();

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
#endif