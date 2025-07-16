#include "ros/ros.h"
#include "serial/serial.h"
#include "tracking/gimbal_serial.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "dynamic_reconfigure/client.h"
#include "costmap_prohibition_layer/CostmapProhibitionLayerConfig.h"
#define FRAME_REFREE_HEADER 0Xa5   
#define FRAME_REFREE_TAILER 0xff   
#define DEBUG_EN            true
serial::Serial gimbal_serial;   
int32_t integer_goal;
bool pub;

uint8_t stuck_trigger = 0;
ros::Time last_movement_time;
// float stuck_time_threshold = 2.0;       
// float stuck_distance_threshold = 0.05;  
tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener* tf_listener= nullptr;
geometry_msgs::TransformStamped last_transform;

gimbal_serial_msg::serial_receive_msg received_msg,last_msg;
gimbal_serial_msg::serial_send_msg send_msg;
geometry_msgs::PoseStamped goal;
geometry_msgs::Twist cmd_vel;
uint8_t tx_buffer[sizeof(send_msg)];  //发送缓冲区

void cmd_velCallback(geometry_msgs::TwistConstPtr msg)
{
    
    // ROS_INFO_STREAM("cmd_vel SENDING \n");
    send_msg.v_x = msg->linear.x;
    send_msg.v_y = msg->linear.y;
    send_msg.v_z = msg->linear.z;
    send_msg.w_z = msg->angular.z;
    // 检查NaN值并处理
    if(isnan(msg->linear.x)) send_msg.v_x= 0;
    if(isnan(msg->linear.y)) send_msg.v_y = 0;
    if(isnan(msg->angular.z)) send_msg.w_z = 0;
    if(isnan(msg->linear.z)) send_msg.v_z = 0;
    std::cout<< "v_x:"<<send_msg.v_x <<std::endl;
    std::cout<< "v_y:"<<send_msg.v_y<<std::endl;
    std::cout<< "v_z:"<<send_msg.v_z <<std::endl;
    std::cout<< "w_z:"<<send_msg.w_z <<std::endl;
}
void GoalCallback(std_msgs::Int32ConstPtr msg){
    integer_goal = msg->data;  
}
void goal_select(geometry_msgs::PoseStamped goal, std::vector<std::vector<double>> goals, int goal_num){
    if(goal_num >= 0 && goal_num < goals.size()){
        auto& arr = goals[goal_num];
        if(arr.size() == 7){
            goal.pose.position.x = arr[0];
            goal.pose.position.y = arr[1];
            goal.pose.position.z = arr[2];
            goal.pose.orientation.x = arr[3];
            goal.pose.orientation.y = arr[4];
            goal.pose.orientation.z = arr[5];
            goal.pose.orientation.w = arr[6];
        }
    }   
}
void pubfuc(const ros::TimerEvent& event)
{
    pub = true;  
}
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv,"gimbal_serial");
    ros::NodeHandle nh("~");
    //ros::Rate loop_rate(500);
    gimbal_serial.setPort("/dev/ttyACM0");  //设置串口设备
    gimbal_serial.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    gimbal_serial.setTimeout(to);
    double stuck_time_threshold;
    double stuck_distance_threshold;
    double goal_publish_timeout;
    int loop_rate_hz;
    std::string team_color;
    int goal_num;
    bool hole_mode;
    dynamic_reconfigure::Client<costmap_prohibition_layer_namespace::CostmapProhibitionLayerConfig> client("/move_base/global_costmap/prohibition_layer");
    nh.param("stuck_time_threshold", stuck_time_threshold, 2.0);
    nh.param("stuck_distance_threshold", stuck_distance_threshold, 0.05);
    nh.param("goal_publish_timeout", goal_publish_timeout, 15.0);
    nh.param("loop_rate", loop_rate_hz, 500);
    nh.param<std::string>("team_color", team_color, "red");
    nh.param<int>("goal_num", goal_num, 11);
    nh.param<bool>("hole_mode", hole_mode, false);
    ros::Rate loop_rate(loop_rate_hz);
    if (access("/dev/ttyACM0", F_OK) == 0)
    {
        try
        {
            gimbal_serial.open();  
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("Unable to open port ");           //打开串口失败，打印信息
            return -1;
        }        
    }
    if(gimbal_serial.isOpen())
    { 
        ROS_INFO_STREAM("Serial Port initialized. \n");        //成功打开串口，打印信息  
    }
    else
    {
        return -1;
    }
    costmap_prohibition_layer_namespace::CostmapProhibitionLayerConfig config;
    // if(client.getCurrentConfiguration(config))
    // {
    //     ROS_INFO("Current enabled: %s", config.enabled ? "true" : "false");
    // }
    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 10, cmd_velCallback);
    //ros::Subscriber escape_vel_sub = nh.subscribe("/escape_vel", 10, cmd_velCallback);
    ros::Subscriber goal_sub = nh.subscribe("/integer_topic", 10, GoalCallback);  //接收整数形式的目标点编号
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);  //用于发送导航目标点到ROS导航栈
    ros::Publisher goal_backup_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_backup",1);  //用于备份发送导航目标点
    ros::Timer pub_timer = nh.createTimer(ros::Duration(goal_publish_timeout),pubfuc);
    send_msg.header = FRAME_REFREE_HEADER;
    send_msg.tailer = FRAME_REFREE_TAILER;
    last_movement_time = ros::Time::now();
    tf_listener = new tf2_ros::TransformListener(tf_buffer);
    try {
        last_transform = tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
        last_movement_time = ros::Time::now();
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF初始化失败: %s", ex.what());
    }
    // 目标点处理（由参数控制）
    std::vector<std::vector<double>> goals;
    if (team_color == "red") {
        for (int i = 0; i < goal_num; ++i) {
            std::vector<double> goal_vec;
            nh.getParam("red_goal" + std::to_string(i), goal_vec);
            goals.push_back(goal_vec);
        }
    } else {
        for (int i = 0; i < goal_num; ++i) {
            std::vector<double> goal_vec;
            nh.getParam("blue_goal" + std::to_string(i), goal_vec);
            goals.push_back(goal_vec);
        }
    }


    while(ros::ok())
    {
        //串口数据接收
        if(gimbal_serial.available() >= sizeof(gimbal_serial_msg::serial_receive_msg) && gimbal_serial.isOpen())
        {
            unsigned char rx_buffer[sizeof(gimbal_serial_msg::serial_receive_msg)];
            int header_pos;
            gimbal_serial.read(rx_buffer,sizeof(gimbal_serial_msg::serial_receive_msg));
            for(int i=0; i<sizeof(gimbal_serial_msg::serial_receive_msg); i++){
                if(rx_buffer[i] == FRAME_REFREE_HEADER)     header_pos = i;
            }
            unsigned char ordered_buffer[sizeof(gimbal_serial_msg::serial_receive_msg)];
            for(int i=0;i<sizeof(gimbal_serial_msg::serial_receive_msg);i++){
                ordered_buffer[i] = rx_buffer[(header_pos+i)%sizeof(gimbal_serial_msg::serial_receive_msg)];
            }
            last_msg = received_msg;
            memcpy(&received_msg,ordered_buffer,sizeof(gimbal_serial_msg::serial_receive_msg));
            std::cout << "goal "<< static_cast<int>(received_msg.goal) << std::endl;
            std::cout << "stuck_flag" << static_cast<int>(stuck_trigger) << std::endl;
            if(last_msg.goal != received_msg.goal) pub = true;
        }
        //stuck判定
        // 获取当前位姿
        try{
            geometry_msgs::TransformStamped current_transform = 
                tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
            
            // 计算位移
            double dx = current_transform.transform.translation.x - last_transform.transform.translation.x;
            double dy = current_transform.transform.translation.y - last_transform.transform.translation.y;
            double distance = sqrt(dx * dx + dy * dy);

            // 更新运动状态
            if (distance > stuck_distance_threshold || 
                sqrt(send_msg.v_x*send_msg.v_x + send_msg.v_y*send_msg.v_y) < 0.05) {
                last_movement_time = ros::Time::now();
                last_transform = current_transform;
                stuck_trigger = 0;
                last_movement_time = ros::Time::now();
                if(hole_mode)
                { 
                    config.enabled = false;
                    config.fill_polygons = true;
                    if (client.setConfiguration(config)) {
                        // ROS_INFO("Configuration updated successfully!");
                    } 
                }
            }

            // 卡住判定
            ros::Duration stuck_duration = ros::Time::now() - last_movement_time;
            if (stuck_duration.toSec() > stuck_time_threshold ) {
                stuck_trigger = 2;
                // last_movement_time = ros::Time::now();
                if(hole_mode)
                { 
                    config.enabled = true;
                    config.fill_polygons = true;
                    if (client.setConfiguration(config)) {
                        ROS_INFO("Configuration updated successfully!");
                    } 
                }
            }
            else if(stuck_duration.toSec() > stuck_time_threshold - 6)
            {
                stuck_trigger = 1;
            }
        }
        catch(tf2::TransformException & ex){
            ROS_WARN("Failed to transform base_link to map: %s", ex.what());
            }
        // 目标点处理
        if((!DEBUG_EN))
        {             
            goal_select(goal, goals, received_msg.goal);
       
            goal.header.frame_id = "map";
            goal.header.stamp = ros::Time::now();
            if(pub == true){
                goal_pub.publish(goal);
                goal_backup_pub.publish(goal);
                pub =false;
            }
            //ROS_INFO("publish goal");
            if(DEBUG_EN){
                std::cout<< "header:"<<static_cast<unsigned int>(received_msg.header) <<std::endl;
                std::cout<< "team:"<<static_cast<unsigned int>(received_msg.team )<<std::endl;
                std::cout<< "goal:"<<static_cast<unsigned int>(received_msg.goal) <<std::endl;
                std::cout<< "tailer:"<<static_cast<unsigned int>(received_msg.tailer) <<std::endl;
            }

        }
        send_msg.stuck_trigger = stuck_trigger;  // 设置标志位
        
        memcpy(tx_buffer,&send_msg,sizeof(send_msg));
        gimbal_serial.write(tx_buffer,sizeof(gimbal_serial_msg::serial_send_msg));
        ros::spinOnce();
        loop_rate.sleep();
    }
    if (tf_listener != nullptr) {
        delete tf_listener;
    }
    return 0;
}

