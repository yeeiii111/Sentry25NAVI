#include <cmath>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

//message filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//message type
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

//tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double PI = 3.1415926;

bool use_gazebo_time = true;
double terrian_VoxelSize = 0.05;
double max_groundHeight = 0.1;
//计算地形平均高度
bool adjustZ = false;
double terrian_RadiusZ = 0.5;
double minTerrianNumZ = 0;
double smooth_RateZ = 0.2;
//计算地形倾斜度 
double adjustIncl = false;
double terrian_RadiusIncl  = 1.5;
double minTerrianNumIncl = 0;
double smooth_RateIncl = 0.2;
double InclFittingThreshold = 0.2;
double maxIncl = 30.0;

const int systemDelay = 5;
int systemInitCount = 0;
bool systemInited  = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr scan_data(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr terrian_cloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr terrian_cloudIncl(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr terrian_cloudDwz(new pcl::PointCloud<pcl::PointXYZI>);

std::vector<int> scan_ind;

ros::Time odom_time;

float vehicle_x = 0;
float vehicle_y = 0;
float vehicle_z = 0;
float vehicle_yaw = 0;
float vehicle_pitch =0;
float vehicle_roll = 0;
float vehicle_yaw_rate = 0;
float vehicle_speed = 0;
double vehicle_height = 0.2;


float terrian_z = 0;
float terrian_roll = 0;
float terrian_pitch = 0;
                     
const int MAX_NUM = 400;
float vehicle_XStack[MAX_NUM];
float vehicle_YStack[MAX_NUM];
float vehicle_ZStack[MAX_NUM];
float vehicle_YawStack[MAX_NUM];
float vehicle_PitchStack[MAX_NUM];
float vehicle_RollStack[MAX_NUM];
float terrian_YawStack[MAX_NUM];
float terrian_RollStack[MAX_NUM];
float terrian_PitchStack[MAX_NUM];
double odom_TimeStack[MAX_NUM];
int odom_SendIDPointer = -1;
int odom_RecIDPointer = 0;

pcl::VoxelGrid<pcl::PointXYZI> terrian_DwzFilter;

ros::Publisher* pub_scan_pointer = NULL;

