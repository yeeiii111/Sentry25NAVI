#include <mutex>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mutex>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
// 下面这个头文件，不包含intellisense就疯狂报错，不知道为什么
// 后发现是mutex必须放在point_cloud.h point_types.h之前，如果放在之后且包含下面的头文件，就不报错但没有代码提示，
// 如果放在之后且不包含，就报错（但是编译没问题），如果放在之前就一切正常，之后还是要将底层的头文件放在前面
// #include <tf/transform_listener.h>
class Relocalization{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    Relocalization();
    ~Relocalization() = default;
    void registration(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, double leaf_size);
    void timer(const ros::TimerEvent& event);
    void Diverge_Callback(const std_msgs::Bool::ConstPtr &msg);
    void Standard_Scan_Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
    void compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, 
                            pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree,
                            pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfh);
    pcl::PointCloud<pcl::PointXYZ> fpfh_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
private:
    bool                                        debug_en;
    bool                                        localize_success;
    std_msgs::Bool                              diverge;
    int                                         freq;
    int                                         pointcloud_count;
    int                                         num_threads;
    int                                         num_neighbors;
    double                                      diverge_threshold;
    double                                      leaf_size;
    double                                      map_leaf_size;
    double                                      max_dist_sq;
    double                                      max_iterations;
    std::string                                 pcd_path;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         scan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         scan_odom;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         aligned;  
    pcl::PointCloud<pcl::PointXYZ>::Ptr         prior_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         high_features_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         high_features_scan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         filtered_prior_map;
    pcl::PointCloud<pcl::PointCovariance>::Ptr  source_cov;
    pcl::PointCloud<pcl::PointCovariance>::Ptr  target_cov;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr  source_fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr  target_fpfh;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree;
    std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree;
    tf2_ros::Buffer                             tf2_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor,small_gicp::ParallelReductionOMP>> register_;
    ros::Subscriber                             scan_sub;
    ros::Subscriber                             initial_pose_sub;
    ros::Subscriber                             diverge_sub;
    ros::Publisher                              target_pub;
    ros::Publisher                              source_pub;
    ros::Publisher                              align_pub;
    ros::Publisher                              diverge_pub;
    ros::Timer                                  run_timer;
    geometry_msgs::TransformStamped             T_map_odom;
    tf2_ros::StaticTransformBroadcaster         broadcaster;
    Eigen::Affine3d                             T_map_scan;
    Eigen::Isometry3d                           T_odom_lidar;
    Eigen::Isometry3d                           previous_icp_result;
    Eigen::Isometry3d                           fpfh_result;
    std::mutex                                  scan_mutex;
    pcl::PointCloud<pcl::Normal>::Ptr           point_normal;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;

};
