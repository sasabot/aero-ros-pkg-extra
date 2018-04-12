#!/usr/bin/env python

import rospy
import time
import math
import threading
from std_msgs.msg import *

mutex = threading.Lock()
angle_goal_ = 0.0

def command_subscriber(data):
    global angle_goal_
    mutex.acquire()
    angle_goal_ = data.data
    mutex.release()

def speed_subscriber(data):
    pass

class position_callback(threading.Thread):
    def __init__(self):
        super(position_callback, self).__init__()
        self.publisher = rospy.Publisher('/sensor_plate_controller/state', Float32, queue_size=10)
        self.angle = 0
    def run(self):
        while not rospy.is_shutdown():
            mutex.acquire()
            if abs(angle_goal_ - self.angle) > 0.001:
                if angle_goal_ > self.angle:
                    self.angle += min((angle_goal_ - self.angle), 5)
                else:
                    self.angle -= min((self.angle - angle_goal_), 5)
            mutex.release()
            self.publisher.publish(Float32(self.angle))
            time.sleep(0.1)

#----------- main -----------------#
if __name__ == "__main__":
    rospy.init_node('sensor_plate_controller')
    command_subscriber = rospy.Subscriber('/sensor_plate_controller/command', Float32, command_subscriber)
    speed_subscriber = rospy.Subscriber('/sensor_plate_controller/speed', Int32, speed_subscriber)
    pos_callback = position_callback()
    pos_callback.start()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
