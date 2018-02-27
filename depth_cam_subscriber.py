"""
By Khang Vu & Sherrie Shen, 2018
Last Modified Feb 20, 2018

The script ...

Dependencies:
- realsense_ros_camera

To use:
- Open Terminal and run the code below:

roslaunch realsense_ros_camera rs_camera.launch

"""
import numpy as np

import cv2
import rospy
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf


class DepthCamTest:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('my_node', anonymous=True)
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback, queue_size=10)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=10)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.align_depth_color_callback,
                         queue_size=10)
        # rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback, queue_size=10)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pointcloud_callback, queue_size=10)
        self.listener = tf.TransformListener()
        self.depth_unit = 0.000124986647279
        self.r = rospy.Rate(10)
        self.rgb_data = None
        self.depth_data = None
        self.align_depth_color_data = None
        self.point_cloud = None

    def rgb_callback(self, data):
        try:
            self.rgb_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def depth_callback(self, data):
        try:
            self.depth_data = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print e

    def align_depth_color_callback(self, data):
        try:
            self.align_depth_color_data = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print e

    def pointcloud_callback(self, data):
        self.point_cloud = data

    def show_rgb(self):
        while self.rgb_data is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            cv2.imshow('', self.rgb_data)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def show_depth(self):
        while self.depth_data is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            d = self.depth_data * self.depth_unit * 1000
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)

            cv2.imshow('', d)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def show_align_depth_color(self):
        while self.align_depth_color_data is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            d = self.align_depth_color_data * self.depth_unit * 1000
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)

            cv2.imshow('', d)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def show_all(self):
        while self.align_depth_color_data is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            d = self.depth_data * self.depth_unit * 1000
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            da = self.align_depth_color_data * self.depth_unit * 1000
            da = cv2.applyColorMap(da.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            dda = np.concatenate((d, da), axis=1)
            ddac = np.concatenate((dda, self.rgb_data), axis=1)
            cv2.imshow('', ddac)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    def pointcloud_test(self):
        while self.point_cloud is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z", "rgb"))
            for i, p in enumerate(gen):
                if i == 480 * 640 / 2 - 640 / 2:
                    x, y, z, rgb = p
                    print " x : %f  y: %f  z: %f rgb: %f" % (x, y, z, rgb)
                    break

            d = self.depth_data * self.depth_unit * 1000
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)

            cv2.imshow('', d)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def depth_difference(self):
        while self.align_depth_color_data is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            print (self.align_depth_color_data - self.depth_data).sum()


if __name__ == '__main__':
    test = DepthCamTest()
    # test.depth_difference()
    # test.show_rgb()
    # test.show_depth()
    # test.show_align_depth_color()
    # test.show_all()
    test.pointcloud_test()
