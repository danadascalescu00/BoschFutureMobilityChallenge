#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from sensor_msgs.msg import Image

import cv2 as cv

class laneDetectionNODE():
    def __init__(self):
        # TODO docstring
        rospy.loginfo('starting laneDetectionNODE - init')
        rospy.init_node('laneDetectionNODE', anonymous=False)
        rospy.loginfo('starting laneDetectionNODE - init')
        rospy.Subscriber("/automobile/image_raw", Image, self._streams)


    def run(self):
        rospy.loginfo('starting laneDetectionNODE')
        rospy.spin()

    def imgmsg_to_cv2(self, img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv

    def _streams(self, msg):
        image = self.imgmsg_to_cv2(msg)
        cv.fastNlMeansDenoisingColored(image, None,10,10,7,21)
        cv.imshow('Test', image)
        cv.waitKey(0)

