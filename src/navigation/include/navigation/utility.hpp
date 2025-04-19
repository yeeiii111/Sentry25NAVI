#ifndef ROBOMASTER_UTILITY_H
#define ROBOMASTER_UTILITY_H
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

double GetEuclideanDistance(const geometry_msgs::PoseStamped & pose_1,
                            const geometry_msgs::PoseStamped & pose_2){
  // std::cout << "pose_1.x: " << pose_1.pose.position.x << std::endl;
  // std::cout << "pose_2.x: " << pose_2.pose.position.x << std::endl;
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
double anglelimit(double angle_in){
  double angle_out;
  if (angle_in > M_PI) angle_out = angle_in - 2*M_PI;
  else if (angle_in < -M_PI) angle_out = angle_in + 2*M_PI;
  else angle_out = angle_in;
  return angle_out;
}
int signum(double num){
  if(num > 0) return 1;
  else if(num < 0) return -1;
  else return 0;
}
double computeCurvature(const geometry_msgs::PoseStamped& p1, 
                        const geometry_msgs::PoseStamped& p2, 
                        const geometry_msgs::PoseStamped& p3) {
    // 计算向量
    double dx1 = p2.pose.position.x- p1.pose.position.x, dy1 = p2.pose.position.y - p1.pose.position.y;
    double dx2 = p3.pose.position.x - p2.pose.position.x, dy2 = p3.pose.position.y - p2.pose.position.y;
    // 计算夹角变化
    double angle_diff = atan2(dy2, dx2) - atan2(dy1, dx1);
    // 曲率 = 1/曲率半径
    if(std::isnan(angle_diff)) ROS_ERROR("curvature NAN");
    return std::abs(angle_diff) / GetEuclideanDistance(p1, p3);
}
double sigmoid(double x)
{
    return 1 / (1 + exp(-x));
}
#endif