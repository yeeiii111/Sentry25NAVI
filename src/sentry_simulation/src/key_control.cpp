#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "termio.h"
#include "string"
#include "thread"

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_SPACE 0x20

#define KEYCODE_W_CAP 0x57
#define KEYCODE_S_CAP 0x53
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
int key_command = 0x00;

void scanKeyboard();
void scanAndPublish();
ros::Publisher cmd_pub;
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "key_control");
    ROS_INFO("key_control started");
    ros::NodeHandle nh;
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    scanAndPublish();
    ros::spin();
    return 0;
}

void scanKeyboard() {
    struct termios new_settings;
    struct termios stored_settings;
    //设置终端参数
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    // system("stty -echo");
    key_command = getchar();
    // system("stty -echo");
    tcsetattr(0, TCSANOW, &stored_settings);
}

void scanAndPublish() {
    ROS_INFO("start to scan keyboard");
    geometry_msgs::Twist cmd_vel;
    while (ros::ok()) {
        scanKeyboard();
        ROS_INFO("get key: %c", (char)key_command);
        switch (key_command) {
        case 'w':
        case 'W':
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 1.0;
            key_command = 0;
            break;
        case 's':
        case 'S':
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = -1.0;
            key_command = 0;
            break;
        case 'a':
        case 'A':
            cmd_vel.angular.z = 1.5;
            cmd_vel.linear.x = 0;
            key_command = 0;
            break;
        case 'd':
        case 'D':
            cmd_vel.angular.z = -1.5;
            cmd_vel.linear.x = 0;
            key_command = 0;
            break;
        case 'x':
        case 'X':
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            key_command = 0;
            break;
        default:
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            break;
        }
        cmd_pub.publish(cmd_vel);
    }

}