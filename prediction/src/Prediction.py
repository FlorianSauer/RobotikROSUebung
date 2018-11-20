#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import tensorflow as tf
from keras import backend as k
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32


class Prediction:
    def __init__(self):
        self.cv_bridge = CvBridge()

def main():
    try:
        # register node
        rospy.init_node('prediction', anonymous=False)

        # init CameraPseudo
        pred = Prediction()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
