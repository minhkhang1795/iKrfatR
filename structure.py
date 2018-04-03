import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
import localization

class Structure(object):

    def __init__(self):
        cubes = []