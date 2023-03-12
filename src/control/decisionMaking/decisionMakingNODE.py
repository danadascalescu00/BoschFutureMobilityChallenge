#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from perception.msg import lineArray, line
from sensor_msgs.msg import Image
import time
import cv2
import sys
import numpy as np
from math import atan, pi

Kp = 1
Ki = 0
Kd = 0

class decisionMakingNODE:
    def __init__(self):
        self.debug = True

        rospy.init_node('decisionMakingNODE', anonymous=False)
        self.command_publisher = rospy.Publisher("/automobile/command", String, queue_size=1)
        self.lane_topic = rospy.Subscriber("/lane_info", lineArray, self._lane)
        self.last_lanes = []
        self.midlane = []
        self.last_frame = None
        self.midlane_history = [(320, 400, 320, 50), (320, 400, 320, 50), (320, 400, 320, 50), (320, 400, 320, 50), (320, 400, 320, 50)] 
        if self.debug:
            rospy.Subscriber("/automobile/image_raw", Image, self._streams)
            cv2.startWindowThread()
            cv2.namedWindow("Debug decision making")
            

    def set_speed(self, spd: float): # spd is float
        command = String()
        command.data = '{"action":"1","speed":' + str(spd) + '}'
        print(command.data)
        self.command_publisher.publish(command)
        time.sleep(0.05)

    def set_steering(self, angle: float): # angle is 0 on center, negative to the left and positive to the right
        command = String()
        command.data = '{"action":"2","steerAngle":' + str(angle) +'}'
        print(command.data)
        self.command_publisher.publish(command)
        time.sleep(0.05)

    def turn90deg(self, direction: bool): # direction = False for left, True for right
        self.set_speed(0.4)
        if not direction:
            time.sleep(2.3)
            self.set_steering(-21)
        else:
            self.set_steering(21)
        time.sleep(7.2)
        self.set_steering(0)

    def stop(self):
        self.set_speed(0)
        self.set_steering(0)

    def imgmsg_to_cv2(self, img_msg):

        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv

    def calculate_midlane(self):
        return ((self.midlane_history[0][0] + self.midlane_history[1][0] + self.midlane_history[2][0] + self.midlane_history[3][0] + self.midlane_history[4][0]) // 5,
                (self.midlane_history[0][1] + self.midlane_history[1][1] + self.midlane_history[2][1] + self.midlane_history[3][1] + self.midlane_history[4][1]) // 5,
                (self.midlane_history[0][2] + self.midlane_history[1][2] + self.midlane_history[2][2] + self.midlane_history[3][2] + self.midlane_history[4][2]) // 5,
                (self.midlane_history[0][3] + self.midlane_history[1][3] + self.midlane_history[2][3] + self.midlane_history[3][3] + self.midlane_history[4][3]) // 5,
                )
            

    def run(self):
        rospy.loginfo('starting decisionMakingNODE')
        time.sleep(2)
        self.stop()
        time.sleep(2)
        self.set_speed(0.5)
        integral = 0
        previous_error = 0
        last_time = round(time.time()*1000) - 20
        
        while not rospy.is_shutdown():
            if self.last_lanes:
                if len(self.last_lanes) == 1:
                    ln = self.last_lanes
                    if ln[0].x1 < 320: # linie din stanga
                        self.midlane = (ln[0].x1 + 200, ln[0].y1, ln[0].x2 + 200, ln[0].y2)
                    else: # linie din dreapta
                        self.midlane = (ln[0].x1 - 200, ln[0].y1, ln[0].x2 - 200, ln[0].y2)
                elif len(self.last_lanes) == 0:
                        self.midlane = (320, 480, 320, 0)   
                
                print(self.midlane)
                error = 320 - ((self.midlane[0] + self.midlane[2]) / 2) # 320 is the half of the width of the frame
                proportional = error
                dt = (round(time.time()*1000) - last_time)
                last_time = round(time.time()*1000)
                integral = integral + error * dt
                derivative = (error - previous_error) / dt
                output = Kp * proportional + Ki * integral + Kd * derivative
                previous_error = error

                m_y = 480 - ((self.midlane[1] + self.midlane[3]) / 2)  # calculating the aprox. distance from the car to the midlane's middle
                angle = round(atan(output / m_y), 2)
                # print(output, m_y)
                
                self.set_steering(-angle / 2 * 180 / pi)

                if self.debug:
                    ln = self.last_lanes
                    if len(self.last_lanes) == 1:
                        cv2.line(self.last_frame, (ln[0].x1, ln[0].y1), (ln[0].x2, ln[0].y2), color=(12,145,255), thickness=7)        
                        
                    elif len(self.last_lanes) > 0:
                        cv2.line(self.last_frame, (ln[0].x1, ln[0].y1), (ln[0].x2, ln[0].y2), color=(12,145,255), thickness=7)
                        cv2.line(self.last_frame, (ln[1].x1, ln[1].y1), (ln[1].x2, ln[1].y2), color=(12,145,255), thickness=7)
                    
                    cv2.line(self.last_frame, (self.midlane[0], self.midlane[1]), (self.midlane[2], self.midlane[3]), color=(120,120,200), thickness=7)
                    
                    self.last_frame = cv2.circle(self.last_frame, (int((self.midlane[0] + self.midlane[2]) / 2),int((self.midlane[1] + self.midlane[3]) / 2)), radius=3, color=(0, 0, 255), thickness=-2)
                    self.last_frame = cv2.circle(self.last_frame, (320,int((self.midlane[1] + self.midlane[3]) / 2)), radius=3, color=(255, 0, 255), thickness=-2)
                    cv2.imshow("Debug decision making", self.last_frame)
                    if cv2.waitKey(20) == 27:
                        sys.exit(0)
            time.sleep(0.1)
        

    def _lane(self, msg):
        self.last_lanes = msg.lines
        ln = self.last_lanes
        if len(self.last_lanes) == 2:
            self.midlane_history.pop()
            self.midlane_history.insert(0, (int((ln[0].x1 + ln[1].x1) / 2), int((ln[0].y1 + ln[1].y1) / 2),
                            int((ln[0].x2 + ln[1].x2) / 2), int((ln[0].y2 + ln[1].y2) / 2)))
            self.midlane = self.calculate_midlane()
            # self.midlane = (int((ln[0].x1 + ln[1].x1) / 2), int((ln[0].y1 + ln[1].y1) / 2),
                            # int((ln[0].x2 + ln[1].x2) / 2), int((ln[0].y2 + ln[1].y2) / 2))


        # print('-' * 20)
        # print(msg.lines)
        # print('-' * 20)

    def _streams(self, msg):
        img = self.imgmsg_to_cv2(msg)
        self.last_frame = img
        # print(img.shape)


if __name__ == "__main__":
    decisionMaking = decisionMakingNODE()
    decisionMaking.run()