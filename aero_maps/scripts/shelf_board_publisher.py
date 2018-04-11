#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
import math
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from aero_maps import ColladaModel, MovableModel

if __name__ == '__main__':
    rospy.init_node('shelf_board_publisher', anonymous = True)
    modeldir = 'package://aero_maps/models/'
    board_height = 1.135

    board = MovableModel(
        'shelf_board',
        [{'filename': modeldir + 'shelf_board.dae',
         'frame_id': 'shelf_board_0_link_2',
         'parent_id': 'shelf_base_link_0',
         'pos': (0, 0, board_height),
         'quat': (0, 0, 0, 1),
         'color': (0.5, 1, 1, 0.5)}]
    )

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        board.publish()
        rate.sleep()
