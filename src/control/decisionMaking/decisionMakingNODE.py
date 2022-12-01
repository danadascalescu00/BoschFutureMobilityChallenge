#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from utils.msg import laneDetection
import time

class decisionMakingNODE:
    def __init__(self):
        rospy.init_node('decisionMakingNODE', anonymous=False)
        self.command_publisher = rospy.Publisher("/automobile/command", String, queue_size=1)
        self.lane_info = rospy.Subscriber("/lane_info", laneDetection, self._lane)

    def move_forward(self):
        command = String()
        command.data = '{"action":"1","speed": 0.5}'
        self.command_publisher.publish(command)
        time.sleep(5)
        command.data = '{"action":"1","speed": 0.0}'
        self.command_publisher.publish(command)

    def run(self):
        rospy.loginfo('starting decisionMakingNODE')
        time.sleep(2)
        self.move_forward()
        rospy.spin()

    def _lane(self, msg):
        rospy.loginfo(msg)

if __name__ == "__main__":
    decisionMaking = decisionMakingNODE()
    decisionMaking.run()