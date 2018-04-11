#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from aero_maps import ColladaModel, MovableModel
from aero_maps.msg import IndexedPoseStamped

if __name__ == '__main__':
    rospy.init_node('onigiri_model_publisher', anonymous = True)
    onigiri_num = rospy.get_param('~onigiri_num', 3)
    modeldir = 'package://aero_maps/models/'
    rate = rospy.Rate(5)

    props = []
    for onigiri_id in range(onigiri_num):
        props.append(
            {'filename': modeldir + 'onigiri.dae',
            'frame_id': 'onigiri_base_link_' + str(onigiri_id),
            'parent_id' : 'shelf_board_1_link_0',
            'pos' : (0.35, -0.15 - (0.12 * onigiri_id), 0),
            'quat' : (0, 0, 0, 1)
        })

    onigiri = MovableModel('onigiri_model', props)

    sub_set_pose = rospy.Subscriber('/onigiri_model/set_pose',
                                    IndexedPoseStamped,
                                    onigiri.set_indexed_pose_callback)

    while not rospy.is_shutdown():
        onigiri.publish()
        rate.sleep()
