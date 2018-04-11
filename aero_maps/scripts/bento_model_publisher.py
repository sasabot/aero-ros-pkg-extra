#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from aero_maps import ColladaModel, MovableModel
from aero_maps.msg import IndexedPoseStamped

if __name__ == '__main__':
    rospy.init_node('bento_model_publisher', anonymous = True)
    bento_num = rospy.get_param('~bento_num', 3)
    modeldir = 'package://aero_maps/models/'
    rate = rospy.Rate(5)
    props = []

    for bento_id in range(bento_num):
        props.append(
            {'filename': modeldir + 'bento.dae',
            'frame_id': 'bento_base_link_' + str(bento_id),
            'parent_id': 'shelf_board_0_link_0',
            'pos': (0.3 - 0.2 * (bento_id % 2), 0, 0.045 * (bento_id / 2)),
            'quat': (0, 0, 0, 1)
         })

    bento = MovableModel('bento_model', props)

    sub_set_pose = rospy.Subscriber('/bento_model/set_pose',
                                    IndexedPoseStamped,
                                    bento.set_indexed_pose_callback)

    while not rospy.is_shutdown():
        bento.publish()
        rate.sleep()
