import rospy
import roslaunch
import sys
import signal
import rospkg
from std_msgs.msg import Bool
class LaunchController:
    def __init__(self):
        self.launches = {}
        self.localize_sub = None
        self.localize_state = False
        self.former_localize_state = False
    def start_launch(self, launch_file_path, launch_name):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,[launch_file_path])
        self.launches[launch_name] = launch
        launch.start()
        rospy.loginfo("Started launch file '%s'", launch_file_path)
    def stop_launch(self,launch_name):
        if launch_name in self.launches:
            self.launches[launch_name].shutdown()
            del self.launches[launch_name]
            rospy.loginfo("Stopping launch file '%s'",launch_name)
    def stop_all_launches(self):
        for launch in self.launches:
            launch.shutdown()
        self.launches = []
        rospy.loginfo("Stopped all launch files")

    def restart_launch(self,launch_file_path, launch_name):
        self.stop_launch(launch_name)
        rospy.sleep(0.5)
        self.start_launch(launch_file_path,launch_name)

    def stop_all_launches(self):
        """停止所有已启动的launch文件"""
        for launch_name in list(self.launches.keys()):
            self.stop_launch(launch_name)

    def topic_callback(self, msg):
        """话题回调函数"""
        self.former_localize_state = self.localize_state
        if msg.data:
            self.localize_state = True
        else:
            self.localize_state = False
        if self.localize_state and (not self.former_localize_state) :
            self.restart_launch(point_lio_path, "point_lio") 
    def signal_handler(self, sig, frame):
        """处理Ctrl+C信号"""
        rospy.loginfo("Ctrl+C detected, stopping all launch files...")
        self.stop_all_launches()
        sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('launch_controller')
    controller = LaunchController()

    # 注册信号处理函数，用于处理Ctrl+C
    signal.signal(signal.SIGINT, controller.signal_handler)

    # 启动一个launch文件
    rospack = rospkg.RosPack()
    package_path = rospack.get_path("point_lio")
    point_lio_path = package_path + "/launch/mapping_mid360.launch"
    controller.start_launch(point_lio_path, "point_lio")

    package_path = rospack.get_path("sentry_simulation")
    livox2rospath = package_path + "/launch/livox2rosmsg.launch"
    controller.start_launch(livox2rospath, "livox2ros") 

    package_path = rospack.get_path("navigation")
    navi_path = package_path + "/launch/simu_navi.launch"
    controller.start_launch(navi_path, "navi") 
    controller_path = package_path +"/launch/controller.launch"
    controller.start_launch(controller_path, "controller") 

    package_path = rospack.get_path("relocalization")
    localize_path = package_path + "/launch/relocalization.launch"
    controller.start_launch(localize_path, "relocalization") 

    package_path = rospack.get_path("segmentation")
    segmentation_path = package_path + "/launch/obstacle_detect.launch"
    controller.start_launch(segmentation_path, "segmentation")     

    # package_path = rospack.get_path("linefit_ground_segmentation_ros")
    # ground_segmentation_path = package_path + "/launch/segmentation.launch"
    #controller.start_launch(ground_segmentation_path, "ground_segmentation") 
    # 订阅一个话题
    controller.localize_sub = rospy.Subscriber("/localize_success", Bool, controller.topic_callback, queue_size= 30);
    # 检查话题值并重新启动launch文件
    rate = rospy.Rate(1000)  # 10 Hz
    while not rospy.is_shutdown():
        # if controller.localize_state and (not controller.former_localize_state) :
        #     controller.restart_launch(point_lio_path, "point_lio")    
        rate.sleep()
    # 停止所有launch文件
    controller.stop_all_launches()