#include <mutex>
#include "tracking/tracking.hpp"
#include "pcl/common/centroid.h"
#include <pcl_conversions/pcl_conversions.h>

Tracker:: Tracker():obstacle (new pcl::PointCloud<pcl::PointXYZ>){
    ros::NodeHandle nh("~");
    nh.param<int>("freq",freq,10);
    nh.param<std::string>("cloud_name",cloud_name,"");
    nh.param<double>("ClusterTolerance",ClusterTolerance,0.1);
    std::cout << cloud_name << std::endl;
    obstacle_sub = nh.subscribe(cloud_name,10,&Tracker::Cloud_Callback,this);
    centroids_pub = nh.advertise<geometry_msgs::PoseArray>("/centroids",10);
    run_timer = nh.createTimer(ros::Duration(1/freq),&Tracker::extract_dynamic_obstacle,this);
}
void Tracker::extract_dynamic_obstacle(const ros::TimerEvent& event){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::lock_guard<std::mutex> lock(obstacle_mutex);
    {
        *local_cloud = *obstacle;
    }
        if(!local_cloud->empty())
        {
            std::vector<pcl::PointIndices>      cluster_indices;//设成成员变量会导致旧索引残留
            tree->setInputCloud(local_cloud);
            obstacle_poses.header.frame_id = "map";
            obstacle_poses.header.stamp = timestamp;           

            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.5);
            ec.setMinClusterSize(5);
            ec.setMaxClusterSize(1000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(local_cloud);
            ec.extract(cluster_indices);        
            obstacle_poses.poses.clear();
            for(const auto& cluster:cluster_indices){
                Eigen::Vector4d centroid;
                pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
                for(const auto& idx:cluster.indices)
                {
                    cluster_cloud.push_back((*local_cloud)[idx]);
                }
                pcl::compute3DCentroid(cluster_cloud, centroid);
                geometry_msgs::Pose pose;
                pose.position.x = centroid[0];
                pose.position.y = centroid[1];
                pose.position.z = centroid[2];

                obstacle_poses.poses.push_back(pose);            
            }
            centroids_pub.publish(obstacle_poses);       
        }

}
void Tracker::Cloud_Callback(const sensor_msgs::PointCloud2ConstPtr &msg){
    std::lock_guard<std::mutex> lock(obstacle_mutex);
    {
        pcl::fromROSMsg(*msg,*obstacle);
        timestamp = msg->header.stamp;        
    }
}
void Tracker::publish_goal(const geometry_msgs::Pose& goal){

}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"tracking");
    Tracker tracker;
    ros::spin();
    return 0;
}