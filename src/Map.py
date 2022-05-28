#!/usr/bin/python3

from sqlalchemy import false, true
import rospy
import rospkg
import tf
from std_msgs.msg import Int16
from sensor_msgs.msg import MultiEchoLaserScan
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped

import time
import yaml


class MapHokuyo:

    def __init__(self):
        r = rospkg.RosPack()
        pwd_params = r.get_path('map_hokuyo')+'/config/params.yaml'

        with open(pwd_params, 'r') as f:
            self.params = yaml.load(f, Loader=yaml.FullLoader)["map_hokuyo"]

        self.sub_lidar = None
        self.sub_motor = None

        self.pub_map = None

        self.pub_lidar_flag = None
        self.pub_motor_flag = None

        self.lidar_flag = 0
        self.motor_flag = 0

    def setting_subscriber(self):
        self.sub_lidar = rospy.Subscriber(
            "/echoes",
            MultiEchoLaserScan,
            self.scanning,
            queue_size=100)

        self.sub_motor = rospy.Subscriber(
            "/dynamixel_mx/pose",
            PointStamped,
            self.mapping,
            queue_size=100)

    def setting_publisher(self):
        self.pub_map = rospy.Publisher(
            "/map_hokuyo/map/pointcloud",
            PointCloud2)

        self.pub_lidar_flag = rospy.Publisher(
            "map_hokuyo/lidar/flag",
            Int16)

        self.pub_motor_flag = rospy.Publisher(
            "map_hokuyo/motor/flag",
            Int16)

    def scanning(self, multiEcho_ros):
        self.lidar_flag = 0
        self.pub_lidar_flag.publish(1)
        self.pub_lidar_flag.publish(0)
        self.lidar_flag = 0

    def mapping(self, point_ros):
        self.motor_flag = 1
        self.pub_lidar_flag.publish(1)
        self.pub_lidar_flag.publish(0)
        self.motor_flag = 0

    def publish_flags():
        pass


if __name__ == '__main__':
    rospy.init_node('map_hokuyo')
    map_hokuyo = MapHokuyo()
    map_hokuyo.setting_publisher()
    map_hokuyo.setting_subscriber()
    rospy.loginfo("\033[1;32m-> Map.\033[0m")
    while not rospy.is_shutdown():
        if map_hokuyo.lidar_flag == 0:
            map_hokuyo.pub_lidar_flag.publish(0)

        if map_hokuyo.motor_flag == 0:
            map_hokuyo.pub_motor_flag.publish(0)
    # rospy.spin()
