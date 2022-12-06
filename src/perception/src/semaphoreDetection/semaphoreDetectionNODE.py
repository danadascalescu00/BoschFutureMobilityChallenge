#!/usr/bin/env python3

import rospy
import numpy as np
import sys
from sensor_msgs.msg import Image

from time import sleep

from transformers import YolosFeatureExtractor, YolosForObjectDetection

from threading import Thread

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

        self.detection_thread = Thread(target=self.processStream)
        self.last_image = None

        cv.startWindowThread()
        cv.namedWindow("Test Semaphore")
        cv.namedWindow("Test color change")

    def run(self):
        rospy.loginfo('starting semaphoreDetectionNODE')
        self.detection_thread.start()
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
        # image = cv.resize(image, (346, 256), interpolation = cv.INTER_CUBIC)
        image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
        self.last_image = image

    def processStream(self):
        while not rospy.is_shutdown():
            image = self.last_image
            if image is not None:
                h, w, c = image.shape

                inputs = self._feature_extractor(images=image, return_tensors="pt")
                outputs = self._object_detection_model(**inputs)
                
                target_sizes = torch.tensor([(h, w)])
                results = self._feature_extractor.post_process_object_detection(outputs, threshold=0.7, target_sizes=target_sizes)[0]

                for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
                    box = [int(round(i, 2)) for i in box.tolist()]
                    print(
                        f"Detected {self._object_detection_model.config.id2label[label.item()]} with confidence "
                        f"{round(score.item(), 3)} at location {box}"
                    )
                    cv.rectangle(image, (box[0], box[1]), (box[2], box[3]), (255, 0, 0), 2)

                    if self._object_detection_model.config.id2label[label.item()] == 'traffic light':
                        traffic_light_cropped = image[box[1]:box[3], box[0]:box[2], :]
                        traffic_light_cropped_hsv = cv.cvtColor(traffic_light_cropped, cv.COLOR_BGR2HSV)

                        kernel = np.ones((5, 5), "uint8")

                        red_lower_limit = np.array([136, 87, 111]) # setting the red lower limit
                        red_upper_limit = np.array([185, 255, 255]) # setting the red upper limit
                        red_mask = cv.inRange(traffic_light_cropped_hsv, red_lower_limit, red_upper_limit)

                        red_mask = cv.dilate(red_mask, kernel)

                        green_lower_limit = np.array([25, 52, 72]) # setting the red lower limit
                        green_upper_limit = np.array([102, 255, 255]) # setting the red upper limit
                        green_mask = cv.inRange(traffic_light_cropped_hsv, green_lower_limit, green_upper_limit)

                        red = cv.bitwise_and(traffic_light_cropped_hsv, traffic_light_cropped_hsv, mask = red_mask)
                        green = cv.bitwise_and(traffic_light_cropped_hsv, traffic_light_cropped_hsv, mask = green_mask)

                        traffic_light_color_mask = red + green
                        print("red mean color value: ", np.mean(traffic_light_color_mask[traffic_light_color_mask > (0., 0., 0.)]))
                        
                        cv.imshow('Test color change', traffic_light_color_mask)
                        


                cv.imshow('Test Semaphore', image)
                if cv.waitKey(20) == 27:
                    sys.exit(0)

if __name__ == "__main__":
    semaphoreDetection = semaphoreDetectionNODE()
    semaphoreDetection.run()