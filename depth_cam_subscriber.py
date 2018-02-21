import numpy as np

import cv2
import rospy
from cv_bridge import CvBridgeError, CvBridge
from realsense_ros_camera.msg import Extrinsics
from sensor_msgs.msg import Image


class DepthCamTest:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('my_node', anonymous=True)
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback, queue_size=10)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=10)
        rospy.Subscriber('/camera/extrinsics/depth_to_color', Extrinsics, self.depth_callback1, queue_size=10)
        self.depth_scale = 0.000124986647279
        self.rgb_data = None
        self.depth_data = None

    def rgb_callback(self, data):
        try:
            self.rgb_data = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

    def depth_callback(self, data):
        # data.encoding = "bgr8"
        try:
            self.depth_data = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print e

    def depth_callback1(self,data):
        try:
            print data
        except CvBridgeError as e:
            print e

    def show_rgb(self):
        while self.rgb_data is None:
            pass

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()
            cv2.imshow('', self.rgb_data)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def show_depth(self):
        while self.depth_data is None:
            pass

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()
            d = self.depth_data
            print d
            d = self.depth_data * self.depth_scale * 1000
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)

            cv2.imshow('', d)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


if __name__ == '__main__':
    test = DepthCamTest()
    # test.show_rgb()
    test.show_depth()
