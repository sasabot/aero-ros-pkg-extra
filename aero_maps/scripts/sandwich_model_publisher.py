#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from aero_maps import ColladaModel, MovableModel
from aero_maps.msg import IndexedPoseStamped

if __name__ == '__main__':
    rospy.init_node('sandwich_model_publisher', anonymous = True)
    sandwich_num = rospy.get_param('~sandwich_num', 5)
    modeldir = 'package://aero_maps/models/'
    rate = rospy.Rate(5)

    props = []
    for sandwich_id in range(sandwich_num):
        props.append(
            {'filename': modeldir + 'sandwich.dae',
            'frame_id': 'sandwich_base_link_' + str(sandwich_id),
            'parent_id': 'shelf_board_1_link_1',
            'pos': (0.35, 0.2 - (0.1 * sandwich_id), 0),
            'quat': (0, 0, 0, 1)
         })

    sandwich = MovableModel('sandwich_model', props)

    sub_set_pose = rospy.Subscriber('/sandwich_model/set_pose',
                                    IndexedPoseStamped,
                                    sandwich.set_indexed_pose_callback)

    while not rospy.is_shutdown():
        sandwich.publish()
        rate.sleep()
