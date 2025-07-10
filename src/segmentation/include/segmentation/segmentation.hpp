#ifndef SEGMENTATION_H
#define SEGMENTATION_H
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include"geometry_msgs/Twist.h"
#include <sensor_msgs/PointCloud2.h>
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
// #include "livox_ros_driver2/CustomMsg.h"
class Obstacle_detector{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Obstacle_detector();
    ~Obstacle_detector() = default;

    void detect(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_global);
    void transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_robot); 
    void timer(const ros::TimerEvent& event);
    // void Livox_Scan_Callback(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
    void Standard_Scan_Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void Obs_Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void Costmap_Callback(const nav_msgs::OccupancyGridConstPtr& msg);
    void grid2pointcloud(const nav_msgs::OccupancyGrid& costmap, pcl::PointCloud<pcl::PointXY>& cloud);
    void cloud_crop(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector3d pos, float box_size, pcl::PointCloud<pcl::PointXYZ>::Ptr &result);
private:
    double                                          distance_threshold = 0.2;
    double                                          costmap_distance_threshold = 0.05;
    double                                          leaf_size;
    double                                          diverge_threshold;
    double                                          high_match_threshold;
    double                                          lidar_height;
    double                                          lidar_roll;
    double                                          lidar_y;
    double                                          max_dist_sq;
    double                                          max_iterations;
    float                                           box_size;
    int                                             min_diverge_num;
    int                                             freq;
    bool                                            high_match;
    bool                                            prior_map_pub_en;
    bool                                            use_livox_cloud;
    bool                                            debug_en;
    bool                                            align_en;
    std_msgs::Bool                                  diverge;
    std_msgs::Bool                                  match;
    pcl::KdTreeFLANN<pcl::PointXYZ>                 kdtree;
    pcl::KdTreeFLANN<pcl::PointXY>                  flat_kdtree;
    std::string                                     map_path;
    std::string                                     hole_path;
    ros::Timer                                      run_timer;
    ros::Time                                       scan_timestamp;
    ros::Time                                       last_scan_timestamp;
    Eigen::Vector3d                                 box_center;
    pcl::PointCloud<pcl::PointXY>::Ptr              costmap_points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             prior_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             hole;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             filtered_prior_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             obstacle;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             obs;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             scan_sensor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             last_scan_sensor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             scan_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             cropped_scan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             cropped_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr             cropped_obs;
    pcl::PointCloud<pcl::PointCovariance>::Ptr      source_cov;
    pcl::PointCloud<pcl::PointCovariance>::Ptr      target_cov;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree;
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor,small_gicp::ParallelReductionOMP>> register_;
    Eigen::Isometry3d                           previous_icp_result;

    std::shared_ptr<tf::TransformListener>          tf_listener;
    ros::Publisher                                  obstacle_pub;
    ros::Publisher                                  prior_map_pub;
    ros::Publisher                                  aligned_pub;
    ros::Publisher                                  diverge_pub;
    ros::Publisher                                  body_frame_pub;
    ros::Publisher                                  match_pub;//点云匹配上一定match，没匹配上可能是里程计退化也可能只是瞬间角速度太大，所以用两个标志位描述定位状态
    ros::Subscriber                                 scan_sub;
    ros::Subscriber                                 obs_sub;
    nav_msgs::OccupancyGrid                         costmap;
    std::chrono::time_point<std::chrono::high_resolution_clock>     match_time;
    Eigen::Isometry3d                               T_baselink_sensor;//sensor中的坐标左乘这个得到在baselink坐标系中的坐标
                                                                      //Pb = T_b_a * Pa
    std::mutex                                      scan_mutex;
    std::mutex                                      obs_mutex;
};
bool Grid2world(int gx, int gy, const nav_msgs::OccupancyGrid& costmap,
                double& wx, double& wy)
{
    if (costmap.info.width == 0 || costmap.info.height == 0) return false;

    wx = (gx+0.5) * costmap.info.resolution + costmap.info.origin.position.x;
    wy = (gy+0.5) * costmap.info.resolution + costmap.info.origin.position.y;

    return true;   
}
#endif