#!/usr/bin/python3

import rospy
import rospkg
import tf
from std_msgs.msg import Header
from std_msgs.msg import Int16
from sensor_msgs import point_cloud2
from sensor_msgs.msg import MultiEchoLaserScan
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped

import time
import yaml
import cv2
import numpy as np
import open3d as o3d


class MapHokuyo:

    def __init__(self):
        r = rospkg.RosPack()
        pwd_params = r.get_path('map_hokuyo')+'/config/params.yaml'

        with open(pwd_params, 'r') as f:
            self.params = yaml.load(f, Loader=yaml.FullLoader)["map_hokuyo"]

        self.map_echo0 = np.array([[0, 0, 0]])
        self.map_echo1 = np.array([[0, 0, 0]])
        self.map_echo2 = np.array([[0, 0, 0]])

        self.map_header = Header()
        self.map_header.frame_id = self.params["frames"]["world"]

        self.sub_lidar = None
        self.sub_motor = None

        self.pub_map_echo0 = None
        self.pub_map_echo1 = None
        self.pub_map_echo2 = None

        self.flag_Echo = False

        self.pub_lidar_flag = None
        self.pub_motor_flag = None

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
        self.pub_map_echo0 = rospy.Publisher(
            "/map_hokuyo/echo0/pointcloud",
            PointCloud2,
            queue_size=10)

        self.pub_map_echo1 = rospy.Publisher(
            "/map_hokuyo/echo1/pointcloud",
            PointCloud2,
            queue_size=10)

        self.pub_map_echo2 = rospy.Publisher(
            "/map_hokuyo/echo2/pointcloud",
            PointCloud2,
            queue_size=10)

        self.pub_lidar_flag = rospy.Publisher(
            "map_hokuyo/lidar/flag",
            Int16)

        self.pub_motor_flag = rospy.Publisher(
            "map_hokuyo/motor/flag",
            Int16)

    def scanning(self, multiEcho_ros):
        self.pub_lidar_flag.publish(1)
        self.multiEcho_ros = multiEcho_ros
        self.flag_Echo = True
        self.pub_lidar_flag.publish(0)

    def mapping(self, pose_motor_ros):
        self.pub_motor_flag.publish(1)
        while self.flag_Echo == False:
            pass
        self.flag_Echo = False
        multiEcho_ros = self.multiEcho_ros

        range_Lidar_ros = multiEcho_ros.ranges
        int_Lidar_ros = multiEcho_ros.intensities

        N = len(range_Lidar_ros)
        tetha = np.linspace(
            multiEcho_ros.angle_min, multiEcho_ros.angle_max, N)

        #             x    y    z    1
        pcd_echo0 = [[0], [0], [0], [1]]
        pcd_echo1 = [[0], [0], [0], [1]]
        pcd_echo2 = [[0], [0], [0], [1]]

        intensities_0 = [0]
        intensities_1 = [0]
        intensities_2 = [0]

        for idx in range(0, N):
            range_tmp = range_Lidar_ros[idx]
            intensities_tmp = int_Lidar_ros[idx]
            N_tmp = len(intensities_tmp.echoes)

            pcd_echo0[0].append(range_tmp.echoes[0]*np.cos(tetha[idx]))
            pcd_echo0[1].append(range_tmp.echoes[0]*np.sin(tetha[idx]))
            pcd_echo0[2].append(0)
            pcd_echo0[3].append(1)

            intensities_0.append(intensities_tmp.echoes[0])

            if N_tmp > 1:
                pcd_echo1[0].append(range_tmp.echoes[1]*np.cos(tetha[idx]))
                pcd_echo1[1].append(range_tmp.echoes[1]*np.sin(tetha[idx]))
                pcd_echo1[2].append(0)
                pcd_echo1[3].append(1)

                intensities_1.append(intensities_tmp.echoes[1])

                if N_tmp > 2:
                    pcd_echo2[0].append(range_tmp.echoes[2]*np.cos(tetha[idx]))
                    pcd_echo2[1].append(range_tmp.echoes[2]*np.sin(tetha[idx]))
                    pcd_echo2[2].append(0)
                    pcd_echo2[3].append(1)

                    intensities_2.append(intensities_tmp.echoes[2])

        pcd_echo0 = np.array(pcd_echo0)
        pcd_echo1 = np.array(pcd_echo1)
        pcd_echo2 = np.array(pcd_echo2)

        rvec = np.radians([0, -pose_motor_ros.point.x, 0])
        tvec = [0, 0, 0]
        tvec[2] += self.params["origins"]["world2motor"]["xyz"][2]
        tvec[2] += self.params["origins"]["motor2lidar"]["xyz"][2]

        mp_velo = np.eye(4)
        mp_velo[:3, :3] = cv2.Rodrigues(rvec)[0]
        mp_velo[:3, 3] = tvec

        map_echo_tmp = mp_velo.dot(pcd_echo0).T[:, :3]
        self.map_echo0 = np.append(self.map_echo0, map_echo_tmp, axis=0)

        map_echo_tmp = mp_velo.dot(pcd_echo1).T[:, :3]
        self.map_echo1 = np.append(self.map_echo1, map_echo_tmp, axis=0)

        map_echo_tmp = mp_velo.dot(pcd_echo2).T[:, :3]
        self.map_echo2 = np.append(self.map_echo2, map_echo_tmp, axis=0)

        self.pub_map_echo0.publish(
            point_cloud2.create_cloud_xyz32(self.map_header, self.map_echo0))

        self.pub_map_echo1.publish(
            point_cloud2.create_cloud_xyz32(self.map_header, self.map_echo1))

        self.pub_map_echo2.publish(
            point_cloud2.create_cloud_xyz32(self.map_header, self.map_echo1))

        self.pub_motor_flag.publish(0)

    def save_mapa(self):
        print("")
        print("save_mapa")
        print("")
        file = "/home/sirui/Pictures/map_echos.txt"

        with open(file, 'w') as f:
            f.write("x y z echo\n")

        for x, y, z in self.map_echo0:
            with open(file, 'a') as f:
                f.write(str(x)+" "+str(y)+" "+str(z)+" "+str(0)+"\n")

        for x, y, z in self.map_echo1:
            with open(file, 'a') as f:
                f.write(str(x)+" "+str(y)+" "+str(z)+" "+str(1)+"\n")

        for x, y, z in self.map_echo2:
            with open(file, 'a') as f:
                f.write(str(x)+" "+str(y)+" "+str(z)+" "+str(2)+"\n")


if __name__ == '__main__':
    rospy.init_node('map_hokuyo')
    map_hokuyo = MapHokuyo()
    map_hokuyo.setting_publisher()
    map_hokuyo.setting_subscriber()
    rospy.loginfo("\033[1;32m-> Map.\033[0m")
    rospy.spin()

    # while not rospy.is_shutdown():
    #     pass
    # map_hokuyo.save_mapa()
