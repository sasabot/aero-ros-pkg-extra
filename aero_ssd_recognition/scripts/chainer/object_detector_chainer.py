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
from cv_bridge import CvBridge
import threading
from sensor_msgs.msg import Image, CameraInfo
from aero_recognition_msgs.msg import Scored2DBox, Scored2DBoxArray
from std_srvs.srv import SetBool, SetBoolResponse

import chainer
from chainercv.links import SSD512
from train_utils import train
from chainercv import utils
from chainercv.visualizations import vis_bbox

from detection_dataset import DetectionDataset, get_class_list

from dynamic_reconfigure.server import Server
from aero_ssd_recognition.cfg import ObjectDetectorConfig

class FCNObjectDetector():

    def __init__(self):

        ## ros params (dynamic reconfigure)
        self.__gpu = None
        self.__model_name = None
        self.__score_thresh = None
        self.__eco_mode = None
        self.__eco_hz = None
        self.__boost_hz = None
        self.__visualize = None

        ## ros service flag
        self.__run_flag = False

        ## internal params
        self.__model_list = {} # list of usable model
        self.__model = None # chainer model
        self.__labels = {} # list of class
        self.__bridge = CvBridge()
        self.__pretrained_model = None
        self.__class_list = None
        self.__label_color = None
        self.__lock = threading.Lock()

        ## eco mode flags
        self.__boost = False
        self.__latest_detect = rospy.Time.now()
        self.__latest_call = rospy.Time.now()

        ## load model list
        list_file = os.path.join(rospkg.RosPack().get_path("aero_ssd_recognition"), "models/chainer/model_list.yaml")
        try:
            model_list = open(list_file, "r")
        except IOError:
            rospy.logfatal("Cannot open model list file!")
        self.__model_list = yaml.load(model_list)

        ##! publisher, subscriber, Serviceserver, Configserver setup
        self.pub_box = rospy.Publisher('~boxes', Scored2DBoxArray, queue_size = 1)
        self.pub_img = rospy.Publisher('~image', Image, queue_size = 1)
        self.setup()


    def run_detector(self, image_msg):

        self.__lock.acquire()
        if not self.__model:
            self.__lock.release()
            return

        cv_img = None
        try:
            cv_img = self.__bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            print (e)
            self.__lock.release()
            return
        if cv_img is None:
            self.__lock.release()
            return

        img = cv_img.copy()
        if img.ndim == 2:
            # reshape (H, W) -> (1, H, W)
            chainer_img = img[np.newaxis]
        else:
            # transpose (H, W, C) -> (C, H, W)
            chainer_img =  img.transpose((2, 0, 1))

        _bboxes, _labels, _scores = self.__model.predict([chainer_img])
        bboxes, labels, scores = _bboxes[0], _labels[0], _scores[0]

        if not len(bboxes):
            rospy.loginfo("not detection")
            if self.__eco_mode:
                ## Publish empty result if eco_mode
                boxes = Scored2DBoxArray()
                boxes.header = image_msg.header
                self.pub_box.publish(boxes)
                if (image_msg.header.stamp - self.__latest_detect).to_sec() > 3.0: ## if nothing detected for last 3 secs, turn off boost mode
                    self.__boost = False
        else:
            rospy.loginfo("found objects : %s" % [ self.__labels[label] for label in labels ])
            if self.__eco_mode:
                self.__boost = True
                self.__latest_detect = image_msg.header.stamp

            ##! publish and visualize
            boxes = Scored2DBoxArray()
            for bbox, label, score in zip(bboxes, labels, scores):
                box = Scored2DBox()
                box.label = self.__labels[label]
                box.x = bbox[1]
                box.y = bbox[0]
                box.width = bbox[3] - bbox[1]
                box.height = bbox[2] - bbox[0]
                box.score = score
                boxes.boxes.append(box)

            boxes.header = image_msg.header
            self.pub_box.publish(boxes)

        im_out = self.draw_label(img, bboxes, labels, scores)

        if self.__visualize:
            cv.namedWindow('detection', cv.WINDOW_NORMAL)
            cv.imshow('detection', im_out)
            cv.waitKey(3)
        self.__lock.release()

        imout_msg = self.__bridge.cv2_to_imgmsg(im_out, "bgr8")
        imout_msg.header = image_msg.header
        self.pub_img.publish(imout_msg)


    def load_model(self):
        try:
            self.__pretrained_model = self.__model_list[self.__model_name]["model"]
            self.__class_list = self.__model_list[self.__model_name]["class_list"]
            self.__labels = get_class_list(self.__class_list)
            self.__model = SSD512(
                n_fg_class=len(self.__labels),
                pretrained_model=self.__pretrained_model)
            if self.__gpu >= 0:
                chainer.cuda.get_device_from_id(self.__gpu).use()
                self.__model.to_gpu()
            self.__model.score_thresh = self.__score_thresh
            return True
        except:
            self.__labels = {}
            self.__model = None
            return False


    def is_name_correct(self):
        if self.__model_name is None:
            rospy.logfatal('PROVIDE MODEL NAME!')
            return False

        if not self.__model_list.has_key(self.__model_name):
            rospy.logfatal('NOT SUCH MODEL')
            return False

        return True


    def draw_label(self, img, bboxes, labels, scores):
        rect_img = cv.resize(img.copy(), (img.shape[1], img.shape[0]))
        im_out = rect_img.copy()
        [
            [
                cv.rectangle(rect_img, (bbox[1], bbox[0]), (bbox[3], bbox[2]), self.__label_color[label], -1),
                cv.rectangle(rect_img, (bbox[1], bbox[0]), (bbox[3], bbox[2]), (0, 0, 255), 4),
                cv.putText(rect_img, self.__labels[label] + ": " + "{0:.2f}".format(score), (int(bbox[1]), int(bbox[0]) - 5), cv.FONT_HERSHEY_TRIPLEX, 1.4, self.__label_color[label], 1, 16)
            ] for bbox, label, score in zip(bboxes, labels, scores)
        ]
        alpha = 0.4
        cv.addWeighted(im_out, alpha, rect_img, 1.0 - alpha, 0, im_out)
        return im_out


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
        self.__eco_mode = config.eco_mode
        self.__eco_hz = config.eco_hz
        self.__boost_hz = config.boost_hz
        self.__visualize = config.visualize

        model_updated = False
        if self.__score_thresh != config.score_thresh:
            self.__score_thresh = config.score_thresh
            model_updated = True

        if self.__model_name != config.model_name:
            self.__model_name = config.model_name
            model_updated = True

        if model_updated:
            if self.is_name_correct():
                rospy.loginfo('MODEL LOAD : %s' % self.__model_name)
                if self.load_model():
                    rospy.loginfo('DETECTOR SETUP SUCCESSFUL')
                    rospy.loginfo('Detect thresh: %s' % self.__model.score_thresh)
                else:
                    rospy.logerr('DETECTOR SETUP FAILED!')
            else:
                rospy.logerr('DETECTOR SETUP FAILED!')

        random.seed(5) #! same seed with image_annotation tool
        rand_list = [random.randint(0, 255) for i in xrange(len(self.__labels) * 3)]
        label_color=[]
        for i in xrange(len(self.__labels)):
            label_color.append(rand_list[i * 3:(i + 1) * 3])
        self.__label_color = np.asarray(label_color, dtype=np.float)
        self.__lock.release()
        return config


    def callback(self, image_msg):
        if(self.__run_flag):
            if (not self.__eco_mode) or\
               (self.__boost and (image_msg.header.stamp - self.__latest_call).to_sec() > 1.0 / self.__boost_hz) or\
               ((image_msg.header.stamp - self.__latest_call).to_sec() > 1.0 / self.__eco_hz):
                self.__latest_call = image_msg.header.stamp
                self.run_detector(image_msg)


    def setup(self):
        rospy.Service('~set_mode', SetBool, self.set_flag)
        Server(ObjectDetectorConfig, self.set_param)
        rospy.Subscriber('image', Image, self.callback, tcp_nodelay=True, queue_size=1)


def main(argv):
    try:
        rospy.init_node('fcn_object_detector', anonymous = False)
        fod = FCNObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logfatal("ros error")
        pass


if __name__ == "__main__":
    main(sys.argv)
