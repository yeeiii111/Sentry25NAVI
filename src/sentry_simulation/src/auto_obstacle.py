import rospy
import math

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

class TwoPointController:
    def __init__(self):
        rospy.init_node('two_point_controller')
        self.namespace = rospy.get_param("~namespace", "obstacle")
        self.pub = rospy.Publisher(f'/{self.namespace}/cmd_vel',Twist,queue_size=10)
        # self.sub = rospy.Subscriber(f'/{self.namespace}/odom',Odometry,self.odom_callback)

        self.targets = [Point(x=1.0, y=0.0, z=0.0), Point(x = 0.0, y = 0, z = 0)]
        self.current_target = 1
        self.distance_tolerance = 0.1
        self.max_linear_speed = rospy.get_param("obstacle_speed",1)
        self.duration = 1/self.max_linear_speed
        self.max_angular_speed =1.0
        self.farward = True

    def timer_callback(self,event):
        self.farward = not self.farward
        twist = Twist()
        if self.farward:
            twist.linear.x = self.max_linear_speed
        else:
            twist.linear.x = -self.max_linear_speed
        self.pub.publish(twist)

    def odom_callback(self,msg):
        current_pos = msg.pose.pose.position
        current_ori = msg.pose.pose.orientation
        target = self.targets[self.current_target]

        dx = target.x - current_pos.x
        dy = target.y - current_pos.y
        distance = math.hypot(dx,dy)

        yaw = self.quaternion_to_yaw(current_ori)
        desired_yaw = math.atan2(dy,dx)
        delta_yaw = desired_yaw - yaw

        twist = Twist()

        if distance > self.distance_tolerance:
            # 调整方向
            if abs(delta_yaw) > 0.1:  # 角度容差
                twist.angular.z = self.max_angular_speed * (delta_yaw / abs(delta_yaw))
            else:
                twist.linear.x = self.max_linear_speed
        else:
            # 到达目标，切换下一个点
            self.current_target = 1 - self.current_target
            rospy.loginfo(f"Target reached! Next target: {self.targets[self.current_target]}")

        self.pub.publish(twist)        

    def quaternion_to_yaw(self, quat):
        # 四元数转yaw角
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return yaw
if __name__== '__main__':
    try:
        controller = TwoPointController()
        timer = rospy.Timer(rospy.Duration(controller.duration),controller.timer_callback)
        rospy.spin()
    except rospy.ROSInternalException:
        pass
