#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
import math
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from aero_maps import ColladaModel, MovableModel

if __name__ == '__main__':
    rospy.init_node('shelf_model_publisher', anonymous = True)
    shelf_num = rospy.get_param('~shelf_num', 2)
    modeldir = 'package://aero_maps/models/'
    # board_height_upper = rospy.get_param('~board_height_upper', 0.95)
    # board_height_lower = rospy.get_param('~board_height_lower', 0.66)

    shelves = []
    subscribers = []
    for shelf_id in range(shelf_num):
        board_height_lower = rospy.get_param(
            '~shelf' + str(shelf_id) + '/board_height_lower', 0.66)
        board_height_upper = rospy.get_param(
            '~shelf' + str(shelf_id) + '/board_height_upper', 0.95)
        shelf_pos = rospy.get_param(
            '~shelf' + str(shelf_id) + '/pos',
            [0.97 - (0.97 * shelf_id), 1, 0])
        shelf_quat = rospy.get_param(
            '~shelf' + str(shelf_id) + '/quat',
            [0, 0, -math.sqrt(0.5), math.sqrt(0.5)])

        shelf_quat_norm = 0.0
        for q in shelf_quat:
            shelf_quat_norm += q * q
        shelf_quat_norm = math.sqrt(shelf_quat_norm)
        for i in range(len(shelf_quat)):
            shelf_quat[i] = shelf_quat[i] / shelf_quat_norm

        shelf = MovableModel(
            'shelf_model_'  + str(shelf_id),
            [
                {'filename': modeldir + 'shelf_base.dae',
                 'frame_id': 'shelf_base_link_' + str(shelf_id),
                 'parent_id': 'map',
                #  'pos': (0.97 - (0.97 * shelf_id), 1, 0),
                 'pos': shelf_pos,
                #  'quat': (0, 0, -math.sqrt(0.5), math.sqrt(0.5)),
                 'quat': shelf_quat,
                 'color': (0.9, 0.9, 0.9, 1)},
                {'filename': modeldir + 'shelf_box.dae',
                 'frame_id': 'shelf_base_link_' + str(shelf_id),
                 'color': (0.9, 0.9, 0.9, 1)},
                {'filename': modeldir + 'shelf_board.dae',
                 'frame_id': 'shelf_board_0_link_' + str(shelf_id),
                 'parent_id': 'shelf_base_link_' + str(shelf_id),
                 'pos': (0, 0, board_height_lower),
                 'quat': (0, 0, 0, 1),
                 'color': (0.5, 1, 1, 0.5)},
                {'filename': modeldir + 'shelf_board.dae',
                 'frame_id': 'shelf_board_1_link_' + str(shelf_id),
                 'parent_id': 'shelf_base_link_' + str(shelf_id),
                 'pos': (0, 0, board_height_upper),
                 'quat': (0, 0, 0, 1),
                 'color': (0.5, 1, 1, 0.5)},
                {'filename': modeldir + 'marker.dae',
                 'frame_id': 'shelf_' + str(shelf_id) + "_marker_link_" + str(shelf_id * 3 + 0),
                 'parent_id': 'shelf_base_link_' + str(shelf_id),
                 'pos': (0.005, 0, 1.2),
                 'quat': (0, 0, 0, 1),
                 'color': (0.0, 0.0, 0.0, 0.5)},
                {'filename': modeldir + 'marker.dae',
                 'frame_id': 'shelf_' + str(shelf_id) + "_marker_link_" + str(shelf_id * 3 + 1),
                 'parent_id': 'shelf_base_link_' + str(shelf_id),
                 'pos': (0.005, -0.33, 1.2),
                 'quat': (0, 0, 0, 1),
                 'color': (0.0, 0.0, 0.0, 0.5)},
                {'filename': modeldir + 'marker.dae',
                 'frame_id': 'shelf_' + str(shelf_id) + "_marker_link_" + str(shelf_id * 3 + 2),
                 'parent_id': 'shelf_base_link_' + str(shelf_id),
                 'pos': (0.005, 0.33, 1.2),
                 'quat': (0, 0, 0, 1),
                 'color': (0.0, 0.0, 0.0, 0.5)},
            ]
        )

        sub_set_pose = rospy.Subscriber('/shelf_' + str(shelf_id) + '/set_pose',
                                        PoseStamped,
                                        shelf.set_pose_callback)

        shelves.append(shelf)
        subscribers.append(sub_set_pose)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        for shelf in shelves:
            shelf.publish()
        rate.sleep()
