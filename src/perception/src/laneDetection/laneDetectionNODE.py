#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from sensor_msgs.msg import Image
from perception.msg import lineArray, line

import cv2 as cv

class laneDetectionNODE():
    def __init__(self):
        # TODO docstring
        rospy.init_node('laneDetectionNODE', anonymous=False)
        rospy.Subscriber("/automobile/image_raw", Image, self._streams)
        self.lane_publisher = rospy.Publisher("/lane_info", lineArray, queue_size=1)

        cv.startWindowThread()
        cv.namedWindow("Test")

    def run(self):
        rospy.loginfo('starting laneDetectionNODE')
        rospy.spin()    

    def imgmsg_to_cv2(self, img_msg):

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
        h, w, _ = image.shape
        image = image[(h // 3):, :, :].copy()
        interest_field_view = image.copy()
        interest_field_view = cv.cvtColor(interest_field_view, cv.COLOR_RGB2GRAY)
        bilateral_filtered_imaged = cv.bilateralFilter(interest_field_view, 10, 75, 75)
        edges = cv.Canny(bilateral_filtered_imaged, 50, 150, apertureSize = 3)

        # Taking a matrix of size 3 as the kernel
        kernel = np.ones((5, 5), np.uint8)

        dilated_edges = cv.dilate(edges, kernel, iterations=1)

        # HoughLinesP method to directly obtain line end points
        
        lines_detected = cv.HoughLinesP(
            dilated_edges, # input edge image,
            1, # distance resolution in pixels
            np.pi / 180, # angle resolution in radians
            threshold = 50, # min number of votes for valid line
            minLineLength = 30, # min allowed length of a single line
            maxLineGap = 10, # max allowed gap between line for joining them together
        )

        message = lineArray()
        # Iterate over points coordinates
        for points in lines_detected:
            x1, y1, x2, y2 = points[0]
            # Draw the lines joining the points
            cv.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            ln = line()
            ln.x1 = x1
            ln.y1 = y1
            ln.x2 = x2
            ln.y2 = y2
            message.lines.append(ln)
        
        self.lane_publisher.publish(message)

        cv.imshow('Test', image)
        if cv.waitKey(20) == 27:
            sys.exit(0)

if __name__ == "__main__":
    laneDetection = laneDetectionNODE()
    laneDetection.run()