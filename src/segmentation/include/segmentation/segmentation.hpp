#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include"geometry_msgs/Twist.h"
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver2/CustomMsg.h"
class Obstacle_detector{
public:
    Obstacle_detector();
    ~Obstacle_detector() = default;

    void detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_global);
    void transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_robot);  
    void timer(const ros::TimerEvent& event);
    void Scan_Callback(const livox_ros_driver2::CustomMsg::ConstPtr &msg);

private:
    double                              distance_threshold = 0.2;
    int                                 freq;
    pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
    std::string                         map_path;
    ros::Timer                          run_timer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prior_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_sensor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_map;
    std::shared_ptr<tf::TransformListener> tf_listener;
    ros::Publisher obstacle_pub;
    ros::Subscriber scan_sub;
    std::vector<double> extrinT;
    std::vector<double> extrinR;
    Eigen::Affine3d T_sensor_baselink;
    std::mutex scan_mutex;
};