#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include"geometry_msgs/Twist.h"
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver2/CustomMsg.h"
class Obstacle_detector{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Obstacle_detector();
    ~Obstacle_detector() = default;

    void detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_global);
    void transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_robot); 
    void timer(const ros::TimerEvent& event);
    void Livox_Scan_Callback(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
    void Standard_Scan_Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
private:
    double                                          distance_threshold = 0.2;
    double                                          leaf_size;
    int                                             freq;
    bool                                            prior_map_pub_en;
    bool                                            use_livox_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ>                 kdtree;
    std::string                                     map_path;
    ros::Timer                                      run_timer;
    ros::Time                                       scan_timestamp;
    ros::Time                                       last_scan_timestamp;
    pcl::PointCloud<pcl::PointXYZ>                  tem;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             prior_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             filtered_prior_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             obstacle;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             scan_sensor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             last_scan_sensor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             scan_map;
    std::shared_ptr<tf::TransformListener>          tf_listener;
    ros::Publisher                                  obstacle_pub;
    ros::Publisher                                  prior_map_pub;
    ros::Subscriber                                 scan_sub;
    std::vector<double>                             extrinT;
    std::vector<double>                             extrinR;
    std::vector<double>                             IMU_extrinT;
    std::vector<double>                             IMU_extrinR;
    Eigen::Isometry3d                               T_baselink_sensor;//sensor中的坐标左乘这个得到在baselink坐标系中的坐标
                                                                      //Pb = T_b_a * Pa
    Eigen::Isometry3d                               T_baselink_IMU;
    Eigen::Isometry3d                               T_map_odom;
    std::mutex                                      scan_mutex;
};