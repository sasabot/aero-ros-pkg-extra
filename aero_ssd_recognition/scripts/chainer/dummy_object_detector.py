#!/usr/bin/env python

import argparse
import matplotlib.pyplot as plot
import yaml

import rospy
import rospkg
import os
import sys
import math
import numpy as np
import random
import cv2 as cv
import threading
from sensor_msgs.msg import Image, CameraInfo
from aero_recognition_msgs.msg import Scored2DBox, Scored2DBoxArray
from std_srvs.srv import SetBool, SetBoolResponse

from dynamic_reconfigure.server import Server
from aero_ssd_recognition.cfg import ObjectDetectorConfig

class DummyObjectDetector():

    def __init__(self):

        ## ros params (dynamic reconfigure)
        self.__gpu = None
        self.__model_name = None
        self.__score_thresh = None
        self.__visualize = None

        ## ros service flag
        self.__run_flag = True

        ## internal params
        self.__model = None # chainer model
        self.__pretrained_model = None
        self.__lock = threading.Lock()

        ##! publisher, subscriber, Serviceserver, Configserver setup
        self.pub_box = rospy.Publisher('~boxes', Scored2DBoxArray, queue_size = 1)
        self.setup()

    def run_detector(self):
        self.__lock.acquire()
        if not self.__model:
            self.__lock.release()
            return

        ## dummy result data
        if self.__model_name == "container":
            bboxes = np.array([[600, 600, 800, 700],
                               [200, 200, 320, 320]])
            labels = ["onigiri", "caffelatte"]
            scores = np.array([0.8, 0.8])
        else: # fcsc
            bboxes = np.array([[600, 600, 800, 700],
                               [200, 200, 320, 320]])
            labels = ["sandwichtamago", "sandwichtamago"]
            scores = np.array([0.8, 0.8])

        ##! publish dummy result
        boxes = Scored2DBoxArray()
        for bbox, label, score in zip(bboxes, labels, scores):
            box = Scored2DBox()
            box.label = label
            box.x = bbox[1]
            box.y = bbox[0]
            box.width = bbox[3] - bbox[1]
            box.height = bbox[2] - bbox[0]
            box.score = score
            boxes.boxes.append(box)

        self.__lock.release()

        boxes.header.stamp = rospy.Time.now()
        boxes.header.frame_id = "ps4eye_frame"
        self.pub_box.publish(boxes)


    def load_model(self):
        try:
            self.__model = True
            return True
        except:
            self.__model = None
            return False


    def is_name_correct(self):
        if self.__model_name is None:
            rospy.logfatal('PROVIDE MODEL NAME!')
            return False

        if False:
            rospy.logfatal('NOT SUCH MODEL')
            return False

        return True


    def set_flag(self, req):
        prev_flag = self.__run_flag
        self.__run_flag = req.data
        if(prev_flag != self.__run_flag):
            if(self.__run_flag):
                rospy.loginfo("Start detection")
            else:
                rospy.loginfo("Stop detection")
        res = SetBoolResponse(success = True)
        res.success = True
        return res

    def set_param(self, config, level):
        self.__lock.acquire()
        rospy.loginfo('Config Update')
        self.__gpu = config.gpu

        self.__score_thresh = config.score_thresh
        self.__visualize = config.visualize

        model_updated = False
        if self.__model_name != config.model_name:
            self.__model_name = config.model_name
            model_updated = True

        if model_updated:
            if self.is_name_correct():
                if self.load_model():
                    rospy.loginfo('DETECTOR SETUP SUCCESSFUL')
                else:
                    rospy.logerr('DETECTOR SETUP FAILED!')
            else:
                rospy.logerr('DETECTOR SETUP FAILED!')

        random.seed(5) #! same seed with image_annotation tool
        self.__lock.release()
        return config

    def callback(self):
        if(self.__run_flag):
            self.run_detector()

    def setup(self):
        rospy.Service('~set_mode', SetBool, self.set_flag)
        Server(ObjectDetectorConfig, self.set_param)


def main(argv):
    try:
        rospy.init_node('dummy_object_detector', anonymous = False)
        dod = DummyObjectDetector()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            dod.callback()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logfatal("ros error")
        pass


if __name__ == "__main__":
    main(sys.argv)
