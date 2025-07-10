#include "ros/ros.h"
#include "serial/serial.h"
#include "tracking/gimbal_serial.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

#define FRAME_REFREE_HEADER 0Xa5
#define FRAME_REFREE_TAILER 0xff
#define DEBUG_EN            true
serial::Serial gimbal_serial;
int32_t integer_goal;
bool pub;
gimbal_serial_msg::serial_receive_msg received_msg,last_msg;
gimbal_serial_msg::serial_send_msg send_msg;
geometry_msgs::PoseStamped goal;
geometry_msgs::Twist cmd_vel;
uint8_t tx_buffer[sizeof(send_msg)];
void cmd_velCallback(geometry_msgs::TwistConstPtr msg)
{
    // ROS_INFO_STREAM("cmd_vel SENDING \n");
    send_msg.v_x = msg->linear.x;
    send_msg.v_y = msg->linear.y;
    send_msg.v_z = msg->linear.z;
    send_msg.w_z = msg->angular.z;
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
void pubfuc(const ros::TimerEvent& event)
{
    pub = true;
}
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv,"gimbal_serial");
    ros::NodeHandle nh;
    ros::Rate loop_rate(500);
    gimbal_serial.setPort("/dev/ttyACM0");
    gimbal_serial.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    gimbal_serial.setTimeout(to);
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
        ROS_INFO_STREAM("Serial Port initialized. \n");         //成功打开串口，打印信息  
    }
    else
    {
        return -1;
    }
    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 10, cmd_velCallback);
    //ros::Subscriber escape_vel_sub = nh.subscribe("/escape_vel", 10, cmd_velCallback);
    ros::Subscriber goal_sub = nh.subscribe("/integer_topic", 10, GoalCallback);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    ros::Publisher goal_backup_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_backup",1);
    ros::Timer pub_timer = nh.createTimer(ros::Duration(15),pubfuc);
    int count = 0;
    send_msg.header = FRAME_REFREE_HEADER;
    send_msg.tailer = FRAME_REFREE_TAILER;
    while(ros::ok())
    {
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
            
            if(last_msg.goal != received_msg.goal) pub = true;
        }
        if((!DEBUG_EN))
        {   
            //RED
            // //home
            if(received_msg.goal == 0)
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 0;
                goal.pose.position.x = 0.042;
                goal.pose.orientation.z = 0.40139;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.91591;
            }
            //supply
            if(received_msg.goal == 1 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = -1.234;
                goal.pose.position.x = 3.893;
                goal.pose.orientation.z = 0.850299;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.526210;
            }

            //point1
            if(received_msg.goal == 11 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 8.01;
                goal.pose.position.x = -2.044;
                goal.pose.orientation.z = 0.8038;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.5948;
            }

            //point2
            if(received_msg.goal == 12 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 10.068;
                goal.pose.position.x = -4.526;
                goal.pose.orientation.z = 0.48288;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.87568;
            }
            //point3
            if(received_msg.goal == 13 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 13.54;
                goal.pose.position.x = -7.882;
                goal.pose.orientation.z = 0.42921;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.90321;
            }      
            //point4
            if(received_msg.goal == 14 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 12.499;
                goal.pose.position.x = -3.636;
                goal.pose.orientation.z = 0.850299;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.526210;
            }

            //fort
            if(received_msg.goal == 15 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 5.191;
                goal.pose.position.x = -1.06;
                goal.pose.orientation.z = 0.8038;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.5948;
            }
            //T
            if(received_msg.goal == 16 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 6.571;
                goal.pose.position.x = -7.645;
                goal.pose.orientation.z = 0.8038;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.5948;
            }    
            if(received_msg.goal == 17 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 4.967;
                goal.pose.position.x = -2.29;
                goal.pose.orientation.z = 0.8038;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.5948;
            }         
            if(received_msg.goal == 18 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 3.533;
                goal.pose.position.x = 1.610;
                goal.pose.orientation.z = 0.8038;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.5948;
            }    
            //enemy side
            if(received_msg.goal == 19 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 16.473;
                goal.pose.position.x = -7.582;
                goal.pose.orientation.z = 0.8038;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.5948;
            }  
            if(received_msg.goal == 20 )   
            {
                goal.pose.position.z = 0.0;
                goal.pose.position.y = 15.777;
                goal.pose.position.x = -1.745;
                goal.pose.orientation.z = 0.8038;
                goal.pose.orientation.y = 0;
                goal.pose.orientation.x = 0;
                goal.pose.orientation.w = 0.5948;
            }  
            //BLUE
            // home
            // if(received_msg.goal == 0)
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 0;
            //     goal.pose.position.x = 0.042;
            //     goal.pose.orientation.z = 0.40139;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.91591;
            // }
            // //supply
            // if(received_msg.goal == 1 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = -2.81;
            //     goal.pose.position.x = 3.796;
            //     goal.pose.orientation.z = 0.850299;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.526210;
            // }

            // //point1
            // if(received_msg.goal == 11 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 9.47;
            //     goal.pose.position.x = 4.246;
            //     goal.pose.orientation.z = 0.8038;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.5948;
            // }

            // //point2
            // if(received_msg.goal == 12 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 7.82;
            //     goal.pose.position.x = -1.52;
            //     goal.pose.orientation.z = 0.48288;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.87568;
            // }
            // //point3
            // if(received_msg.goal == 13 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 7.08;
            //     goal.pose.position.x = 1.44;
            //     goal.pose.orientation.z = 0.42921;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.90321;
            // }      
            // //point4
            // if(received_msg.goal == 14 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 12.54;
            //     goal.pose.position.x = 1.527;
            //     goal.pose.orientation.z = 0.850299;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.526210;
            // }

            // //fort
            // if(received_msg.goal == 15 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 4.8;
            //     goal.pose.position.x = -0.07;
            //     goal.pose.orientation.z = 0.8038;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.5948;
            // }
            // //T
            // if(received_msg.goal == 16 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 7.48408;
            //     goal.pose.position.x = -6.3537;
            //     goal.pose.orientation.z = 0.8038;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.5948;
            // }      
            // if(received_msg.goal == 17 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 4.474;
            //     goal.pose.position.x = -1.7;
            //     goal.pose.orientation.z = 0.8038;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.5948;
            // }        
            // if(received_msg.goal == 18 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 3.286;
            //     goal.pose.position.x = 2.013;
            //     goal.pose.orientation.z = 0.8038;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.5948;
            // }  
            // //enemy side
            // if(received_msg.goal == 19 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 17.566;
            //     goal.pose.position.x = -4.535;
            //     goal.pose.orientation.z = 0.8038;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.5948;
            // }  
            // if(received_msg.goal == 20 )   
            // {
            //     goal.pose.position.z = 0.0;
            //     goal.pose.position.y = 15.63;
            //     goal.pose.position.x = 1.76;
            //     goal.pose.orientation.z = 0.8038;
            //     goal.pose.orientation.y = 0;
            //     goal.pose.orientation.x = 0;
            //     goal.pose.orientation.w = 0.5948;
            // }  
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
        // count ++;
        // if(count>=10)
        // {
        //     count = 0;
        // }
        memcpy(tx_buffer,&send_msg,sizeof(send_msg));
        gimbal_serial.write(tx_buffer,sizeof(gimbal_serial_msg::serial_send_msg));
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
