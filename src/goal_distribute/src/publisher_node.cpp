#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "Client.h" 
#include <string.h>

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "int_publisher_node");
    ros::NodeHandle nh;

    // 创建一个发布者，话题名为 "integer_topic"，队列大小为 10
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("integer_topic", 10);

    // 设置发布频率（1Hz）
    ros::Rate rate(1);
    decision_pack pack;

    int count = 0;

    

    while (ros::ok())
    {
        // 创建消息

        Client client(8787);
        char buf[128];
        client.Read(buf,128);
        memcpy(&pack,buf,sizeof(decision_pack));

        std_msgs::Int32 msg;
        msg.data = pack.goal;

        // 发布消息
        pub.publish(msg);

        // 输出日志信息
        ROS_INFO("Published: %d goal : %d config %d", count,pack.goal,pack.config);

        // 按照设定频率休眠
        rate.sleep();
        ++count;
    }

    return 0;
}