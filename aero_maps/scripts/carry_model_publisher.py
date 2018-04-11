#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
import math
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from aero_maps import ColladaModel, MovableModel

if __name__ == '__main__':
    rospy.init_node('carry_model_publisher', anonymous = True)
    modeldir = 'package://aero_maps/models/'
    rate = rospy.Rate(5)
    props = [{'filename': modeldir + 'carry.dae',
              'frame_id': 'carry_base_link',
              'parent_id': 'map',
              'pos': (-1.6, 0.25, 0),
              'quat': (0, 0, math.sqrt(0.5), math.sqrt(0.5)),
              'color': (0.5, 0.5, 0.5, 1)},
             {'filename': modeldir + 'container_part0.dae',
              'frame_id': 'container_base_link',
              'parent_id': 'carry_base_link',
              'pos': (0, 0, 0.7),
              'quat': (0, 0, math.sqrt(0.5), math.sqrt(0.5)),
              'color': (1, 0.3, 0, 1)},
             {'filename': modeldir + 'container_part1.dae',
              'frame_id': 'container_base_link',
              'color': (1, 0.8, 0, 1)}]

    carry = MovableModel('carry_model', props)

    sub_set_pose = rospy.Subscriber('/carry_model/set_pose',
                                    PoseStamped,
                                    carry.set_pose_callback)

    while not rospy.is_shutdown():
        carry.publish()
        rate.sleep()
