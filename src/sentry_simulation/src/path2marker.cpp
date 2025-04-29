#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher marker_pub;

void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    visualization_msgs::MarkerArray markers;
    
    // 先删除之前的标记（可选）
    visualization_msgs::Marker delete_marker;
    delete_marker.ns = "path2marker";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // 为每个路径点生成一个球体标记
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header = msg->header;
        marker.ns = "path2marker";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = msg->poses[i].pose;
        marker.scale.x = 0.01;  // 球体直径
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.r = 1.0;   // 红色
        marker.color.a = 1.0;   // 不透明度
        markers.markers.push_back(marker);
    }
    
    marker_pub.publish(markers);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path2marker");
    ros::NodeHandle nh ("~");
    std::string subtopic, pubtopic;
    nh.param<std::string>("subtopic",subtopic,"default_topic");
    nh.param<std::string>("pubtopic",pubtopic,"default_topic");   
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(pubtopic, 10);
    ros::Subscriber path_sub = nh.subscribe(subtopic, 10, &pathCallback);
    
    ros::spin();
    return 0;
}