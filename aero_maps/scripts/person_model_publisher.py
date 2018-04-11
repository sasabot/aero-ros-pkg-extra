#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Kazuhiro Sasabuchi JSK

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from aero_maps import ColladaModel, MovableModel
from aero_maps.msg import IndexedPoseStamped

if __name__ == '__main__':
    rospy.init_node('person_model_publisher', anonymous = True)
    person_num = rospy.get_param('~person_num', 1)
    modeldir = 'package://aero_maps/models/'
    rate = rospy.Rate(1)

    props = []
    for person_id in range(person_num):
        props.append(
            {
                'filename': modeldir + 'blue_person.dae',
                'frame_id': 'person_head_link_' + str(person_id),
                'parent_id': 'map',
                'pos': (1.6, -0.6, 1.4),
                'quat': (0, 0, 1, 0)
            }
        )

    people = MovableModel('person_model', props)

    sub_set_pose = rospy.Subscriber('/person_model/set_pose',
                                    IndexedPoseStamped,
                                    people.set_indexed_pose_callback)

    while not rospy.is_shutdown():
        people.publish()
        rate.sleep()
