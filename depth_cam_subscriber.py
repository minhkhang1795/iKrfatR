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
import numpy

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
MID_ROW = 236
MID_COL = 310
DEPTH_UNIT = 0.124986647279


class DepthCamTest:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('my_node', anonymous=True)
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback, queue_size=10)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=10)
        rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pointcloud_callback, queue_size=10)
        self.r = rospy.Rate(10)
        self.rgb_data = None
        self.depth_data = None
        self.align_depth_color_data = None
        self.point_cloud = None
        self.coordinate = {}

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
            d = self.depth_data * DEPTH_UNIT
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
            d = self.depth_data * DEPTH_UNIT
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            da = self.align_depth_color_data * self.depth_unit * 1000
            da = cv2.applyColorMap(da.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            dda = np.concatenate((d, da), axis=1)
            ddac = np.concatenate((dda, self.rgb_data), axis=1)
            cv2.imshow('', ddac)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    def get_coord_from_pixel(self, pixel):
        while self.point_cloud is None:
            print "No point cloud found"

        row, col = pixel
        if row < 0 or col < 0 or row >= IMAGE_HEIGHT or col >= IMAGE_WIDTH:
            print "Row {} and Col {} are invalid".format(row, col)
            return

        points_gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"))
        for i, p in enumerate(points_gen):
            if i == row * IMAGE_WIDTH + col:
                x, y, z = p
                x *= DEPTH_UNIT
                y *= DEPTH_UNIT
                z *= DEPTH_UNIT
                return [x, y, z]

    def get_coords_from_pixels(self, pixels):
        while self.point_cloud is None:
            print "No point cloud found"

        list = []
        for i, pixel in enumerate(pixels):
            row, col = pixel
            if row < 0 or col < 0 or row >= IMAGE_HEIGHT or col >= IMAGE_WIDTH:
                print "Row {} and Col {} are invalid".format(row, col)
                continue
            list.append(row * IMAGE_WIDTH + col)

        coords = []
        count = 0
        points_gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"))
        for i, p in enumerate(points_gen):
            if i in list:
                x, y, z = p
                count += 1
                x *= DEPTH_UNIT
                y *= DEPTH_UNIT
                z *= DEPTH_UNIT
                if not np.math.isnan(x):
                    coords.append([x, y, z])
                if count == len(list):
                    return coords
        return coords

    def rowcol_to_i(self, row, col):
        return row * IMAGE_WIDTH + col

    def i_to_rowcol(self, i):
        return i / IMAGE_WIDTH, i % IMAGE_WIDTH

    def pointcloud_test(self):
        while self.point_cloud is None:
            pass

        while not rospy.is_shutdown():
            self.r.sleep()
            gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z", "rgb"))
            for i, p in enumerate(gen):
                x, y, z, rgb = p
                # if abs(x) < 0.00001:
                # if i == 640*439 + 310:
                if i == 640 * 439 + 350:
                    x *= DEPTH_UNIT
                    y *= DEPTH_UNIT
                    z *= DEPTH_UNIT
                    print "i:%d | x : %f  y: %f  z: %f" % (i, x, y, z)
                    break

            d = self.depth_data * DEPTH_UNIT
            d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)
            cd = np.concatenate((d, self.rgb_data), axis=1)
            cv2.imshow('', cd)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    def find_height_angle(self):
        while self.point_cloud is None:
            pass

        self.r.sleep()
        gen = pc2.read_points(self.point_cloud, field_names=("x", "y", "z"))
        for i, p in enumerate(gen):
            x, y, z = p
            x *= DEPTH_UNIT
            y *= DEPTH_UNIT
            z *= DEPTH_UNIT
            row, col = self.i_to_rowcol(i)
            self.coordinate[(row, col)] = [x, y, z]
        a = self.coordinate[(MID_ROW, MID_COL)]
        b = self.coordinate[(MID_ROW + MID_ROW / 2, MID_COL)]
        ab = self.distance(a, b)
        oa = a[2]
        ob = b[2]
        angle = numpy.arccos((ab ** 2 + oa ** 2 - ob ** 2) / (2 * oa * ab)) * 180 / numpy.pi
        height = numpy.sin(angle * numpy.pi / 180) * oa
        return 90 - angle, height

    def distance(self, a, b):
        return numpy.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


if __name__ == '__main__':
    test = DepthCamTest()

    # test.show_rgb()
    # test.show_depth()
    # test.show_all()
    # test.pointcloud_test()
    # coord = test.get_coord_from_pixel([0, -3])
    # coord = test.get_coords_from_pixels([[479, 639], [0, 0], [0, 4], [2, 0], [5, 6], [-3, -5]])
    while True:
        print test.find_height_angle()
    # print len(coord)
