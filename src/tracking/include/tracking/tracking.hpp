#include <pcl/segmentation/extract_clusters.h>
#include "geometry_msgs/PoseArray.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
class Tracker{
public:
    Tracker();
    ~Tracker() = default;
    void publish_goal(const geometry_msgs::Pose& goal);
    void extract_dynamic_obstacle(const ros::TimerEvent& event); 
    void Cloud_Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
private:
    int                                 freq;
    std::string                         cloud_name;
    double                              ClusterTolerance;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle;
    std::vector<Eigen::Vector4d>        centroids;//质心齐次坐标
    geometry_msgs::PoseArray            obstacle_poses; 
    ros::Subscriber                     obstacle_sub;
    ros::Publisher                      centroids_pub;
    ros::Time                           timestamp;
    ros::Timer                          run_timer;
    std::mutex                          obstacle_mutex;
};
