#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
import math
from visualization_msgs.msg import Marker, MarkerArray
from aero_maps import ColladaModel, MovableModel

if __name__ == '__main__':
    rospy.init_node('container_model_publisher', anonymous = True)
    container_id = rospy.get_param('~container_id', 0)
    modeldir = 'package://aero_maps/models/'
    rate = rospy.Rate(1)
    props = [{'filename': modeldir + 'container_part0.dae',
              'frame_id': 'container_base_link_' + str(container_id),
              'parent_id': 'carry_base_link',
              'pos': (0, 0, 0.7),
              'quat': (0, 0, math.sqrt(0.5), math.sqrt(0.5)),
              'color': (1, 0.3, 0, 1)},
             {'filename': modeldir + 'container_part1.dae',
              'frame_id': 'container_base_link_' + str(container_id),
              'color': (1, 0.8, 0, 1)}]
    container = MovableModel('container_model_' + str(container_id), props)

    while not rospy.is_shutdown():
        container.publish()
        rate.sleep()
