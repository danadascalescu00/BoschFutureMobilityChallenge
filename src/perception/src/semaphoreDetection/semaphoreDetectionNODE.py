#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from sensor_msgs.msg import Image

from time import sleep

from transformers import YolosFeatureExtractor, YolosForObjectDetection

import cv2 as cv
import torch

class semaphoreDetectionNODE():
    def __init__(self):
        # TODO docstring
        rospy.init_node('semaphoreDetectionNODE', anonymous=False)
        rospy.Subscriber("/automobile/image_raw", Image, self._streams)
        # self.lane_publisher = rospy.Publisher("/lane_info", lineArray, queue_size=1)

        self._feature_extractor = YolosFeatureExtractor.from_pretrained('hustvl/yolos-tiny')
        self._object_detection_model = YolosForObjectDetection.from_pretrained('hustvl/yolos-tiny')
        sleep(5)

        cv.startWindowThread()
        cv.namedWindow("Test Semaphore")

    def run(self):
        rospy.loginfo('starting semaphoreDetectionNODE')
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
        image = cv.resize(image, (256, 256), interpolation = cv.INTER_CUBIC)
        # image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        h, w, c = image.shape

        inputs = self._feature_extractor(images=image, return_tensors="pt")
        outputs = self._object_detection_model(**inputs)
        
        target_sizes = torch.tensor([(h, w)])
        results = self._feature_extractor.post_process_object_detection(outputs, threshold=0.9, target_sizes=target_sizes)[0]

        for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
            box = [int(round(i, 2)) for i in box.tolist()]
            print(
                f"Detected {self._object_detection_model.config.id2label[label.item()]} with confidence "
                f"{round(score.item(), 3)} at location {box}"
            )
            cv.rectangle(image, (box[0], box[1]), (box[2], box[3]), (255, 0, 0), 2)

        # h, w, _ = image.shape
        # image = image[(h // 3):, :, :].copy()
        # interest_field_view = image.copy()
        # interest_field_view = cv.cvtColor(interest_field_view, cv.COLOR_RGB2GRAY)
        # bilateral_filtered_imaged = cv.bilateralFilter(interest_field_view, 10, 75, 75)
        # edges = cv.Canny(bilateral_filtered_imaged, 50, 150, apertureSize = 3)

        # # Taking a matrix of size 3 as the kernel
        # kernel = np.ones((5, 5), np.uint8)

        # dilated_edges = cv.dilate(edges, kernel, iterations=1)

        # # HoughLinesP method to directly obtain line end points
        
        # lines_detected = cv.HoughLinesP(
        #     dilated_edges, # input edge image,
        #     1, # distance resolution in pixels
        #     np.pi / 180, # angle resolution in radians
        #     threshold = 50, # min number of votes for valid line
        #     minLineLength = 30, # min allowed length of a single line
        #     maxLineGap = 10, # max allowed gap between line for joining them together
        # )

        # message = lineArray()
        # # Iterate over points coordinates
        # for points in lines_detected:
        #     x1, y1, x2, y2 = points[0]
        #     # Draw the lines joining the points
        #     cv.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        #     ln = line()
        #     ln.x1 = x1
        #     ln.y1 = y1
        #     ln.x2 = x2
        #     ln.y2 = y2
        #     message.lines.append(ln)
        
        # self.lane_publisher.publish(message)

        cv.imshow('Test Semaphore', image)
        if cv.waitKey(20) == 27:
            sys.exit(0)

if __name__ == "__main__":
    semaphoreDetection = semaphoreDetectionNODE()
    semaphoreDetection.run()