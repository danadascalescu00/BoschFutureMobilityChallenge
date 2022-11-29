#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from sensor_msgs.msg import Image

import cv2 as cv

class laneDetectionNODE():
    def __init__(self):
        # TODO docstring
        rospy.init_node('laneDetectionNODE', anonymous=False)
        rospy.Subscriber("/automobile/image_raw", Image, self._streams)

        cv.startWindowThread()
        cv.namedWindow("Test")

    def run(self):
        rospy.loginfo('starting laneDetectionNODE')
        rospy.spin()    

    def imgmsg_to_cv2(self, img_msg):
        # if img_msg.encoding != "bgr8":
            # rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
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
        image = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
        image = cv.Canny(image,50,150,apertureSize = 3)
        
        cv.imshow('Test', image)
        if cv.waitKey(20) == 27:
            sys.exit(0)

if __name__ == "__main__":
    laneDetection = laneDetectionNODE()
    laneDetection.run()