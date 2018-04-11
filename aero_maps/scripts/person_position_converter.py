#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Kazuhiro Sasabuchi JSK

# converts position published from negomo to IndexedPoseStamped

import rospy
# from geometry_msgs.msg import PoseArray
import numpy as np
import math
from geometry_msgs.msg import Pose, Quaternion
from visualization_msgs.msg import Marker
from aero_maps.msg import IndexedPoseStamped

def MarkerCallback(msg):
    if msg.id >= num_heads:
        return
    elif len(msg.points) != 2:
        rospy.logwarn("[person_position_converter] detected size %d", len(msg.points))
        return
    v1 = [1, 0, 0]
    v2 = [msg.points[1].x - msg.points[0].x, msg.points[1].y - msg.points[0].y, msg.points[1].z - msg.points[0].z]
    v = np.cross(v1, v2)
    if np.linalg.norm(v) < 0.0000001:
        if v2[0] < 0:
            ori = Quaternion(0, 0, 1, 0)
        else:
            ori = Quaternion(0, 0, 0, 1)
    else:
        w = math.sqrt(np.linalg.norm(v1)**2 * np.linalg.norm(v2)**2) + np.dot(v1, v2)
        ori = np.array([v[0], v[1], v[2], w])
        norm = np.linalg.norm(ori)
        ori = Quaternion(v[0]/norm, v[1]/norm, v[2]/norm, w/norm)
    pub_set_pose.publish(header=msg.header,
                         index=msg.id,
                         pose=Pose(position=msg.points[0], orientation=ori))

if __name__ == '__main__':
    rospy.init_node('person_position_converter')
    num_heads = rospy.get_param('~person_num', 1)
    sub_positions = rospy.Subscriber('/visualization_marker',
                                     Marker, MarkerCallback, queue_size=1)
    pub_set_pose = rospy.Publisher('/person_model/set_pose',
                                   IndexedPoseStamped, queue_size=10)
    rospy.spin()
