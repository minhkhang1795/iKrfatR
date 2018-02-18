import cv2
import rospy
from std_msgs.msg import UInt8MultiArray


class DepthCamTest:
    def __init__(self):
        rospy.init_node('my_node', anonymous=True)
        rospy.Subscriber('/rbg_data', UInt8MultiArray, self.rgb_callback, queue_size=10)
        rospy.Subscriber('/depth_data', UInt8MultiArray, self.depth_callback, queue_size=10)
        self.rgb_data = None
        self.depth_data = None

    def rgb_callback(self, data):
        self.rgb_data = data

    def depth_callback(self, data):
        self.depth_data = data

    def show_rgb(self):
        while self.rgb_data is None:
            pass
        while not rospy.is_shutdown():
            # print self.rgb_data.data
            # cv2.imshow('', self.rgb_data.data.reshape(480, 640, 3))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def show_depth(self):
        while self.depth_data is None:
            pass
        while not rospy.is_shutdown():
            cv2.imshow('', self.depth_data.data.reshape(480, 640, 3))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


if __name__ == '__main__':
    test = DepthCamTest()
    test.show_rgb()
    # test.show_depth()
