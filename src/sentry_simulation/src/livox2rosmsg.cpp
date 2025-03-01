#include "livox_ros_driver2/CustomMsg.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "sensor_msgs/PointCloud2.h"
ros::Publisher  rosmsg_pub;
void livox2rosmsg(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    int pl_size = msg->point_num;
    pcl::PointCloud<pcl::PointXYZ> pl_full , pl_surf;
    pl_full.resize(pl_size);
    pl_surf.reserve(pl_size);
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
        pl_surf.push_back(pl_full[i]);
      }
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(pl_surf, ros_cloud);
    ros_cloud.header = msg->header;
    rosmsg_pub.publish(ros_cloud);
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"livox2rosmsg");
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    scan_sub = nh.subscribe("/livox/lidar",5,&livox2rosmsg);
    rosmsg_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_ros",5);
    ros::spin();
    return 0;
}
