#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Int16
import time
import localization


class Digital_Cube(object):

    def __init__(self):
        self.height = 0
        self.connections = 0
        self.x = 0
        self.y = 0
        self.z = 0
