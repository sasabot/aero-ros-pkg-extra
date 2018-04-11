#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from aero_maps import ColladaModel, MovableModel
from aero_maps.msg import IndexedPoseStamped

if __name__ == '__main__':
    rospy.init_node('coffee_model_publisher', anonymous = True)
    coffee_num = rospy.get_param('~coffee_num', 3)
    modeldir = 'package://aero_maps/models/'
    rate = rospy.Rate(5)

    props = []
    for coffee_id in range(coffee_num):
        props.append(
            {
                'filename': modeldir + 'coffee.dae',
                'frame_id': 'coffee_base_link_' + str(coffee_id),
                'parent_id': 'shelf_board_1_link_0',
                'pos': (0.35, 0.35 - 0.08 * coffee_id, 0),
                'quat': (0, 0, 0, 1)
            }
        )

    coffee = MovableModel('coffee_model', props)

    sub_set_pose = rospy.Subscriber('/coffee_model/set_pose',
                                    IndexedPoseStamped,
                                    coffee.set_indexed_pose_callback)

    while not rospy.is_shutdown():
        coffee.publish()
        rate.sleep()
