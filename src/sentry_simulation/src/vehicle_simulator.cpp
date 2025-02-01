#include <sentry_simulation/vehicle_simulator.h>

void LivoxCloudCallback(sensor_msgs::PointCloud2::ConstPtr livoxcloud_in)
{
    //system init
    if(!systemInited){
        systemInitCount++;
        if(systemInitCount > systemDelay){
            systemInited = true;
        }
        return;
    }

    double scan_time = livoxcloud_in->header.stamp.toSec();

    if(odom_SendIDPointer < 0){
        return;
    }
    while(odom_TimeStack[(odom_RecIDPointer + 1) % MAX_NUM] < scan_time &&
          odom_RecIDPointer != (odom_SendIDPointer + 1) % MAX_NUM)
    {
        odom_RecIDPointer = (odom_RecIDPointer + 1) % MAX_NUM;
    }


    double odom_RecTime = odom_time.toSec();
    float vehicle_RecX = vehicle_x;
    float vehicle_RecY = vehicle_y;
    float vehicle_RecZ = vehicle_z;
    float vehicle_RecYaw = vehicle_yaw;
    float vehicle_RecRoll = vehicle_roll;
    float vehicle_RecPitch = vehicle_pitch;
    float terrian_RecRoll = terrian_roll;
    float terrian_RecPitch = terrian_pitch;
    
    if(use_gazebo_time)
    {
        odom_RecTime = odom_TimeStack[odom_RecIDPointer];
        vehicle_RecX = vehicle_XStack[odom_RecIDPointer];
        vehicle_RecY = vehicle_YStack[odom_RecIDPointer];
        vehicle_RecZ = vehicle_ZStack[odom_RecIDPointer];
        vehicle_RecYaw = vehicle_YawStack[odom_RecIDPointer];
        vehicle_RecRoll = vehicle_RollStack[odom_RecIDPointer];
        vehicle_RecPitch = vehicle_PitchStack[odom_RecIDPointer];
        terrian_RecRoll = terrian_RollStack[odom_RecIDPointer];
        terrian_RecPitch = terrian_PitchStack[odom_RecIDPointer];
    }
    
    scan_data->clear();
    pcl::fromROSMsg(*livoxcloud_in, *scan_data);
    pcl::removeNaNFromPointCloud(*scan_data, *scan_data, scan_ind);

    for(int i = 0; i< scan_data->points.size(); ++i)
    {
        float point_x1 = scan_data->points[i].x;
        float point_y1 = scan_data->points[i].y * cos(terrian_RecRoll) - scan_data->points[i].z * sin(terrian_RecPitch);
        float point_z1 = scan_data->points[i].y * sin(terrian_RecRoll) + scan_data->points[i].z * cos(terrian_RecPitch);
        float point_x2 = point_x1 * cos(terrian_RecPitch) + point_z1 * sin(terrian_RecPitch);
        float point_y2 = point_y1;
        float point_z2 = point_z1 * cos(terrian_RecPitch) - point_x1 * sin(terrian_RecPitch);
        float point_x3 = point_x2 + vehicle_RecX;
        float point_y3 = point_y2 + vehicle_RecY;
        float point_z3 = point_z2 + vehicle_RecZ;

        scan_data->points[i].x = point_x3;
        scan_data->points[i].y = point_y3;
        scan_data->points[i].z = point_z3;
    }


    sensor_msgs::PointCloud2 scan_data2;
    pcl::toROSMsg(*scan_data, scan_data2);
    scan_data2.header.stamp= ros::Time().fromSec(odom_RecTime);
    scan_data2.header.frame_id = "map";
    pub_scan_pointer->publish(scan_data2); 
    ROS_INFO_THROTTLE(10,"111111");   
}

void TerrianCloudCallback(sensor_msgs::PointCloud2::ConstPtr terrian_cloud2)
{
    if(!adjustZ && !adjustIncl){
        return;
    }

    terrian_cloud->clear();
    pcl::fromROSMsg(*terrian_cloud2, *terrian_cloud);
    pcl::PointXYZI point;
    double elevMean = 0;
    int elevCount = 0;
    bool terrian_valid = true;

    for(int i =0; i < terrian_cloud->points.size(); ++i)
    {
        point = terrian_cloud->points[i];
        float dis = sqrt((point.x - vehicle_x) * (point.x  - vehicle_x) + (point.y - vehicle_y) * (point.y - vehicle_y));
        //地形平均高度
        if(dis < terrian_RadiusZ){
            if(point.intensity < max_groundHeight){
                elevMean += point.z;
                elevCount++;
            } 
            else{
                terrian_valid = false;
            }
        }        
        //地形倾斜度
        if(dis < terrian_RadiusIncl && point.intensity < max_groundHeight){
            terrian_cloudIncl->push_back(point);
        }
    }

    if(elevCount >= minTerrianNumZ)
        elevMean /= elevCount;
    else
        terrian_valid = false;
    
    if(terrian_valid && adjustZ){   //平滑因子
        terrian_z = (1- smooth_RateZ) * terrian_z + smooth_RateZ * elevMean;
    }

    terrian_cloudDwz->clear();
    terrian_DwzFilter.setInputCloud(terrian_cloudIncl);
    terrian_DwzFilter.filter(*terrian_cloudDwz);

    if(terrian_cloudDwz->points.size() < minTerrianNumIncl && !terrian_valid){
        return ;
    }
    
    cv::Mat matA(terrian_cloudDwz->points.size(), 2, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(2, terrian_cloudDwz->points.size(), CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(terrian_cloudDwz->points.size(), 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0));
    
    int inlier_num = 0;
    matX.at<float>(0, 0) = terrian_pitch;
    matX.at<float>(1, 0) = terrian_roll;
    //5次迭代拟合
    for(int iterCount = 0; iterCount < 5; ++iterCount)
    {
        int outlierCount = 0;
        for(int i = 0; i< terrian_cloudDwz->points.size(); ++i)
        {
            matA.at<float>(i, 0) = vehicle_x - point.x;
            matA.at<float>(i, 1) = point.y - vehicle_y;
            matB.at<float>(i, 0) = point.z - elevMean; 

            if(fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0) + matA.at<float>(i, 1) * matX.at<float>(1, 0) -
                    matB.at<float>(i, 0)) > InclFittingThreshold && iterCount > 0)
            {
                matA.at<float>(i, 0) = 0;
                matA.at<float>(i, 1) = 0;
                matB.at<float>(i, 0) = 0;
                outlierCount++;
            }
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if(inlier_num == terrian_cloudDwz->points.size() - outlierCount)
            break;
        inlier_num = terrian_cloudDwz->points.size() - outlierCount;
    }

    if(inlier_num < minTerrianNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0 ||
        fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0)
    {
        terrian_valid = false;
    }

    if(terrian_valid && adjustIncl)
    {
        terrian_pitch = (1.0 - smooth_RateIncl) * terrian_pitch + smooth_RateIncl * matX.at<float>(0, 0);
        terrian_roll = (1.0 - smooth_RateIncl) * terrian_roll + smooth_RateIncl * matX.at<float>(1, 0);
    }    
}

void SpeedCallback(geometry_msgs::TwistStamped::ConstPtr speed_msg)
{
    vehicle_speed = speed_msg->twist.linear.x;
    vehicle_yaw_rate = speed_msg->twist.angular.z;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "vehicle_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.getParam("use_gazebo_time", use_gazebo_time);
    nh_private.getParam("vehicle_height", vehicle_height);
    nh_private.getParam("vehicle_x", vehicle_x);
    nh_private.getParam("vehicle_y", vehicle_y);
    nh_private.getParam("vehicle_z", vehicle_z);
    nh_private.getParam("vehicle_yaw", vehicle_yaw);
    nh_private.getParam("terrian_z", terrian_z);
    nh_private.getParam("terrian_VoxelSize",  terrian_VoxelSize);
    nh_private.getParam("max_groundHeight", max_groundHeight);
    nh_private.getParam("adjustZ", adjustZ);
    nh_private.getParam("terrian_RadiusZ", terrian_RadiusZ);
    nh_private.getParam("minTerrianNumZ", minTerrianNumZ);
    nh_private.getParam("smooth_RateZ", smooth_RateZ);
    nh_private.getParam("adjustIncl", adjustIncl);
    nh_private.getParam("terrian_RadiusIncl", terrian_RadiusIncl);
    nh_private.getParam("minTerrianNumIncl", minTerrianNumIncl);
    nh_private.getParam("smooth_RateIncl", smooth_RateIncl);
    nh_private.getParam("InclFittingThreshold", InclFittingThreshold);
    nh_private.getParam("maxIncl", maxIncl);

    ros::Subscriber sub_livoxcloud = nh.subscribe<sensor_msgs::PointCloud2>("/livox_horizon_points", 2, LivoxCloudCallback);
    ros::Subscriber sub_terriancloud = nh.subscribe<sensor_msgs::PointCloud2>("/terrian_map", 2, TerrianCloudCallback);
    ros::Subscriber sub_speed = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 5, SpeedCallback);

    ros::Publisher pub_vehicleOdom = nh.advertise<nav_msgs::Odometry>("/state_estimation", 5);
    ros::Publisher pub_modelstate = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    ros::Publisher pub_scan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 2);
    pub_scan_pointer = &pub_scan;
    
    nav_msgs::Odometry odom_data;
    odom_data.header.frame_id = "map";
    odom_data.header.frame_id = "/odom";

    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform odom_trans;
    odom_trans.frame_id_ = "/map";
    odom_trans.child_frame_id_ = "/odom";

    gazebo_msgs::ModelState robot_state;
    robot_state.model_name = "robot";
    
    terrian_DwzFilter.setLeafSize(terrian_VoxelSize, terrian_VoxelSize, terrian_VoxelSize);

    std::cout << "Simulation started." << std::endl;

    ros::Rate loop_rate(200);

    bool status = ros::ok();
    while(status)
    {
        ros::spinOnce();

        float vehicle_RecRoll = vehicle_roll;
        float vehicle_RecPitch = vehicle_pitch;
        float vehicle_RecZ = vehicle_z;

        vehicle_roll = terrian_roll * cos(vehicle_yaw) + terrian_pitch * sin(vehicle_yaw);
        vehicle_pitch = -terrian_roll * sin(vehicle_yaw) + terrian_pitch * cos(vehicle_yaw);
        vehicle_yaw += vehicle_yaw_rate * 0.01;
        if(vehicle_yaw > PI)
            vehicle_yaw -= 2 * PI;
        else if (vehicle_yaw < -PI)
            vehicle_yaw += 2 * PI;
        
        vehicle_x = 0.01 * cos(vehicle_yaw) * vehicle_speed;
        vehicle_y = 0.01 *  sin(vehicle_yaw) * vehicle_speed;
        vehicle_z = terrian_z + vehicle_height;

        odom_time = ros::Time::now();

        odom_SendIDPointer = (odom_SendIDPointer + 1) % MAX_NUM;
        odom_TimeStack[odom_SendIDPointer] = odom_time.toSec();
        vehicle_XStack[odom_SendIDPointer] = vehicle_x;
        vehicle_YStack[odom_SendIDPointer] = vehicle_y;
        vehicle_ZStack[odom_SendIDPointer] = vehicle_z;
        vehicle_RollStack[odom_SendIDPointer] = vehicle_roll;
        vehicle_PitchStack[odom_SendIDPointer] = vehicle_pitch;
        vehicle_YawStack[odom_SendIDPointer] = vehicle_yaw;
        terrian_RollStack[odom_SendIDPointer] = terrian_roll;
        terrian_PitchStack[odom_SendIDPointer] = terrian_pitch;

        geometry_msgs::Quaternion getQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicle_roll, vehicle_pitch, vehicle_yaw);

        //200 Hz
        odom_data.header.stamp = odom_time;
        odom_data.pose.pose.position.x = vehicle_x;
        odom_data.pose.pose.position.y = vehicle_y;
        odom_data.pose.pose.position.z = vehicle_z;
        odom_data.pose.pose.orientation = getQuat;
        odom_data.twist.twist.linear.x = vehicle_speed;
        odom_data.twist.twist.linear.z = 200 * (vehicle_z - vehicle_RecZ);
        odom_data.twist.twist.angular.x = 200 * (vehicle_roll - vehicle_RecRoll);
        odom_data.twist.twist.angular.y = 200 * (vehicle_pitch - vehicle_RecPitch);
        odom_data.twist.twist.angular.z = vehicle_yaw_rate;
        pub_vehicleOdom.publish(odom_data);

        odom_trans.stamp_ = odom_time;
        odom_trans.setRotation(tf::Quaternion(getQuat.x, getQuat.y, getQuat.z, getQuat.w));
        odom_trans.setOrigin(tf::Vector3(vehicle_x, vehicle_y, vehicle_z));
        tf_broadcaster.sendTransform(odom_trans);

        robot_state.pose.orientation = getQuat;
        robot_state.pose.position.x = vehicle_x;
        robot_state.pose.position.y = vehicle_y;
        pub_modelstate.publish(robot_state);

        status = ros::ok();
        loop_rate.sleep();
    }
    return 0;
}