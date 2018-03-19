import numpy as np
import time

import cv2
import rospy
from std_msgs.msg import Int32MultiArray, UInt8MultiArray, MultiArrayDimension

import pyrealsense as pyrs


# from pyrealsense.constants import rs_option


class DepthCam:
    def __init__(self):
        # logging.basicConfig(level=logging.INFO)
        rospy.init_node('depth_cam', anonymous=True)
        self.depth_pub = rospy.Publisher("/depth_data", UInt8MultiArray, queue_size=10)
        self.rbg_pub = rospy.Publisher("/rbg_data", UInt8MultiArray, queue_size=10)

    def run(self):
        r = rospy.Rate(10)
        with pyrs.Service() as serv:
            with serv.Device() as dev:

                cnt = 0
                last = time.time()
                smoothing = 0.9
                fps_smooth = 30

                while True:
                    cnt += 1
                    if (cnt % 10) == 0:
                        now = time.time()
                        dt = now - last
                        fps = 10 / dt
                        fps_smooth = (fps_smooth * smoothing) + (fps * (1.0 - smoothing))
                        last = now

                    dev.wait_for_frames()
                    c = dev.color
                    c = cv2.cvtColor(c, cv2.COLOR_RGB2BGR)
                    d = dev.depth * dev.depth_scale * 1000
                    d = cv2.applyColorMap(d.astype(np.uint8), cv2.COLORMAP_RAINBOW)
                    print(dev.depth_scale)
                    # self.rbg_pub.publish(self.build_multi_array(c))
                    # self.depth_pub.publish(self.build_multi_array(d))

                    r.sleep()

    def build_multi_array(self, array=None):
        # let's build a 3x3 matrix:
        mat = UInt8MultiArray()
        mat.data = [0] * array.size

        product = 1
        for dim in range(array.ndim):
            mat.layout.dim.append(MultiArrayDimension())
            mat.layout.dim[dim].label = 'dim'
            mat.layout.dim[dim].size = array.shape[dim]
            mat.layout.dim[dim].stride = array.size/product
            product *= array.shape[dim]
            mat.layout.data_offset = 0

        for i in range(array.shape[0]):
            for j in range(array.shape[1]):
                for k in range(array.shape[2]):
                    mat.data[mat.layout.dim[1].stride * i + mat.layout.dim[2].stride * j + k] = array[i][j][k]
        # mat.data = array.flatten().tolist()
        return mat


if __name__ == '__main__':
    depth_cam = DepthCam()
    depth_cam.run()
