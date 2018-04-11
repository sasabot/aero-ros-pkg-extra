#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @brief 2D shelf box model for scan matching
## @author Hiroaki Yaguchi

import yaml
import math
import numpy
import rospy
import rospkg
import tf
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Pose, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_srvs.srv import Empty, EmptyResponse

## utilities
## @brief make quaternion from theta
## @param theta rotation angle arround z axis
def make_quaternion(theta):
    return [0, 0, math.sin(theta * 0.5), math.cos(theta * 0.5)]

## @brief get rotation angle (z) from quaternion
## @param quat quaternion
def rotation_from_quaternion(quat):
    theta = math.acos(quat[3]) * 2.0
    if quat[2] < 0:
        return -theta
    else:
        return theta

## @brief make transformation from (pos, quat)
## @param pos position
## @param quat rotation
def make_transformation(pos, quat):
    tmat = tf.transformations.quaternion_matrix(quat)
    tmat[0, 3] = pos[0]
    tmat[1, 3] = pos[1]
    tmat[2, 3] = pos[2]
    return tmat

## @brief transform point by matrix
def transform_point(pt, tmat):
    hpt = numpy.array([pt[0], pt[1], pt[2], 1.0])
    tpt = tmat.dot(hpt)
    return tpt[:3]

## @brief distance between point and edge
## @param p point
## @param v0 vertex of edge
## @param v1 other vertex of edge
## @param nv (v1 - v0)
## @param le |nv|
def distance_point_edge(p, v0, v1, nv, le):
    d0 = p - v0
    sq = d0.dot(nv) / (le * le)  # le = nv.norm()
    if sq > 0.0 and sq < 1.0:
        q = v0 + sq * nv
        return math.sqrt(q.dot(q))
    else:
        d1 = p - v1
        return min(math.sqrt(d0.dot(d0)), math.sqrt(d1.dot(d1)))


## @brief manage shelf box and scan
class ShelfBox:
    ## @brief constructor
    ## @param x_pos_list list of shelf vertex position in x axes
    ## @param y_pos_list list of shelf vertex position in y axes
    ## @param shelf_base_link base_link of shelf
    def __init__(self, x_pos_list, y_pos_list, shelf_base_link):
        self.shelf_base_link = shelf_base_link
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.tf_listener.waitForTransform(
            "/map",
            self.shelf_base_link,
            rospy.Time(),
            rospy.Duration(10.0))
        self.shelf_pos, self.shelf_quat = self.tf_listener.lookupTransform(
            "/map",
            self.shelf_base_link,
            rospy.Time())
        self.shelf_theta = rotation_from_quaternion(self.shelf_quat)
        #print self.shelf_pos, self.shelf_quat, self.shelf_theta

        # save original shelf pose
        self.original_shelf_pos = self.shelf_pos
        self.original_shelf_quat = self.shelf_quat
        self.original_shelf_theta = self.shelf_theta

        # vertices
        self.pos_list = []
        self.scan_list = []
        self.mapgrid_list = []
        self.laser_height = 0.2344

        for i in range(len(x_pos_list)):
            point = numpy.array([x_pos_list[i],
                                 y_pos_list[i],
                                 self.laser_height])
            self.pos_list.append(point)

        # create edge
        self.nv_list, self.edge_length_list =  self.create_edge(self.pos_list)

        # marker
        self.markers = MarkerArray()
        self.vertices_to_marker_array()

        # initialpose
        self.initposemsg = PoseWithCovarianceStamped()
        self.initposemsg.pose.covariance[0] = 0.25
        self.initposemsg.pose.covariance[7] = 0.25
        self.initposemsg.pose.covariance[35] = 0.06853891945200942
        # (pi/12.0)^2

        # pointcloud
        self.pc_header = Header(frame_id = 'map', stamp = rospy.get_time())
        self.pc_fields = [
            PointField(name = 'x', offset = 0,
                       datatype = PointField.FLOAT32, count = 1),
            PointField(name = 'y', offset = 4,
                       datatype = PointField.FLOAT32, count = 1),
            PointField(name = 'z', offset = 8,
                       datatype = PointField.FLOAT32, count = 1),
            PointField(name = 'rgb', offset = 12,
                       datatype = PointField.UINT32, count = 1)
        ]
        self.pc_msg = pc2.create_cloud(
            self.pc_header, self.pc_fields, [])

        # mapgrid
        self.mapgrid_header = Header(frame_id = 'map',
                                     stamp = rospy.get_time())
        self.mapgrid_msg = pc2.create_cloud(
            self.mapgrid_header, self.pc_fields, [])

        # publisher
        self.pub_marker = rospy.Publisher('shop_models',
                                          MarkerArray, queue_size = 1)
        self.pub_points = rospy.Publisher('global_scan',
                                          PointCloud2, queue_size = 1)
        self.pub_mapgrid = rospy.Publisher('mapgrid_points',
                                           PointCloud2, queue_size = 1)
        self.pub_initpose = rospy.Publisher('initialpose',
                                            PoseWithCovarianceStamped,
                                            queue_size = 1)

        # subscriber
        self.sub_scan = rospy.Subscriber('scan', LaserScan,
                                         self.scan_to_points)
        self.sub_map = rospy.Subscriber('map', OccupancyGrid,
                                         self.map_to_points)

        # service
        self.srv_update = rospy.Service('~update_shelf_pose', Empty,
                                        self.update_shelf_pose_srv)

        self.srv_initpose = rospy.Service('~update_init_pose', Empty,
                                        self.update_init_pose_srv)

    ## @brief create edge information of shelf box
    def create_edge(self, pos_list):
        nv_list = []
        edge_length_list = []
        for i in range(len(pos_list) - 1):
            v0 = pos_list[i]
            v1 = pos_list[i + 1]
            nv = v1 - v0
            nv_list.append(nv)
            edge_length_list.append(math.sqrt(nv.dot(nv)))
        return nv_list, edge_length_list

    ## @brief create transformed edge information
    def create_transformed_edge(self, pos_list, x, y, theta):
        trans = (x, y, 0)
        quat = make_quaternion(theta)
        tmat = make_transformation(trans, quat)

        pos_list = []
        for pos in self.pos_list:
            pos_list.append(transform_point(pos, tmat))
        nv_list, edge_length_list = self.create_edge(pos_list)
        return pos_list, nv_list, edge_length_list

    ## @brief map to points
    ## @param msg nav_msgs/OccupancyGrid
    def map_to_points(self, msg):
        map_info = msg.info
        cr = math.cos(math.acos(map_info.origin.orientation.w) * 2.0)
        sr = math.sqrt(1.0 - cr * cr)
        self.mapgrid_list = []
        point_list = []
        for y in range(map_info.height):
            yidx = y * map_info.width
            my = y * map_info.resolution + map_info.origin.position.y
            for x in range(map_info.width):
                val = msg.data[x + yidx]
                if val >= 90:
                    mx = x * map_info.resolution + map_info.origin.position.x
                    pt = numpy.array([mx * cr - my * sr,
                                      mx * sr + my * cr,
                                      self.laser_height])
                    if ((math.fabs(pt[0] - self.shelf_pos[0]) < 0.5) and
                        (math.fabs(pt[1] - self.shelf_pos[1]) < 0.5)):
                        self.mapgrid_list.append(pt)
                        point_list.append(
                            [pt[0], pt[1], pt[2], 0x00ff00])

        self.mapgrid_msg = pc2.create_cloud(
            self.mapgrid_header, self.pc_fields, point_list)
        #print(len(self.mapgrid_list))
        self.sub_map.unregister()

    ## @brief LaserScan to points
    ## @param msg sensor_msgs/LaserScan
    def scan_to_points(self, msg):
        try:
            pos, quat = self.tf_listener.lookupTransform(
                "/map",
                msg.header.frame_id,
                rospy.Time())

            tmat = make_transformation(pos, quat)
            #print tmat

            self.scan_list = []
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_inc = msg.angle_increment

            angle = angle_min

            point_list = []

            for r in msg.ranges:
                point = numpy.array([math.cos(angle) * r,
                                     math.sin(angle) * r,
                                     0.0,
                                     1.0])
                g_point = tmat.dot(point)
                self.scan_list.append(g_point.flat[:3])

                #print g_point
                #print point

                point_list.append(
                    [g_point[0], g_point[1], g_point[2], 0xff0000])

                angle += angle_inc

            self.pc_header.stamp = msg.header.stamp
            self.pc_msg = pc2.create_cloud(
                self.pc_header, self.pc_fields, point_list)
            self.pub_points.publish(self.pc_msg)

        except Exception:
            None

    ## @brief scan matching: calc distance between points and shelf box
    def scan_matching(self, scan_list, pos_list, nv_list, edge_length_list):
        dist_sum = 0.0
        for p in scan_list:
            dist_list = []
            for i in range(len(pos_list) - 1):
                v0 = pos_list[i]
                v1 = pos_list[i + 1]
                nv = nv_list[i]
                le = edge_length_list[i]
                dist = distance_point_edge(p, v0, v1, nv, le)
                dist_list.append(dist)
            mindist = min(dist_list)
            # dist_sum -= mindist
            # dist_sum += (1.0 / math.exp(mindist * 10))
            dist_sum += (1.0 / ((mindist + 0.1) * (mindist + 0.1)))
        return dist_sum / len(scan_list)

    ## @brief scan matching with transformation
    def transformed_scan_matching(self, scan_list, pos_list, x, y, theta):
        tpos_list, nv_list, edge_length_list = self.create_transformed_edge(pos_list, x, y, theta)
        return self.scan_matching(
            scan_list, tpos_list, nv_list, edge_length_list)

    ## @brief calc new shelf pose
    def update_shelf_pose(self, scan_list, pos_list):
        shelf_pos_x = self.shelf_pos[0]
        shelf_pos_y = self.shelf_pos[1]
        shelf_theta = self.shelf_theta

        dx = 0.03
        dy = 0.03
        dt = 0.06

        prev_dist = self.transformed_scan_matching(
            scan_list, pos_list, shelf_pos_x, shelf_pos_y, shelf_theta)

        print "(x y theta) = (" + str(shelf_pos_x) + " " + str(shelf_pos_y) + " " + str(shelf_theta) + ")"

        #while prev_dist - dist_o > 1e-3:
        for i in range(10):
            dist_o = self.transformed_scan_matching(
                scan_list, pos_list,
                shelf_pos_x, shelf_pos_y, shelf_theta)
            dist_x = self.transformed_scan_matching(
                scan_list, pos_list,
                shelf_pos_x + dx, shelf_pos_y, shelf_theta)
            dist_xm = self.transformed_scan_matching(
                scan_list, pos_list,
                shelf_pos_x - dx, shelf_pos_y, shelf_theta)
            dist_y = self.transformed_scan_matching(
                scan_list, pos_list,
                shelf_pos_x, shelf_pos_y + dy, shelf_theta)
            dist_ym = self.transformed_scan_matching(
                scan_list, pos_list,
                shelf_pos_x, shelf_pos_y - dy, shelf_theta)
            dist_t = self.transformed_scan_matching(
                scan_list, pos_list,
                shelf_pos_x, shelf_pos_y, shelf_theta + dt)
            dist_tm = self.transformed_scan_matching(
                scan_list, pos_list,
                shelf_pos_x, shelf_pos_y, shelf_theta - dt)

            grad_x = (dist_x - dist_xm) / dx if ( dist_x > dist_o or dist_xm > dist_o ) else (dist_x - dist_xm) / dx / 10.0
            grad_y = (dist_y - dist_ym) / dy if ( dist_y > dist_o or dist_ym > dist_o ) else (dist_y - dist_ym) / dy / 10.0
            grad_t = (dist_t - dist_tm) / dt if ( dist_t > dist_o or dist_tm > dist_o ) else (dist_t - dist_tm) / dt / 10.0

            shelf_pos_x += (grad_x * 0.001)
            shelf_pos_y += (grad_y * 0.001)
            shelf_theta += (grad_t * 0.001)

            print "update grad x:" + str(grad_x) + " y:" + str(grad_y) + " theta:" + str(grad_t)
            print "dist: " + str(prev_dist) + " -> " + str(dist_o)
            prev_dist = dist_o

            print "(x y theta) = (" + str(shelf_pos_x) + " " + str(shelf_pos_y) + " " + str(shelf_theta) + ")"

        return shelf_pos_x, shelf_pos_y, shelf_theta

    def update_init_pose(self, x, y, theta):
        try:
            ## get current base_link pose
            base_current_pos, base_current_quat = self.tf_listener.lookupTransform(
                "/map",
                "/base_link",
                rospy.Time(0))
            base_current_theta = rotation_from_quaternion(base_current_quat)

            print "current x:" + str(base_current_pos[0]) + "  y:" + str(base_current_pos[0]) + "  theta:" + str(base_current_theta)

            diff_theta = self.original_shelf_theta - theta
            diff_x = self.original_shelf_pos[0] - x
            diff_y = self.original_shelf_pos[1] - y
            print "diff x:" + str(diff_x) + "  y:" + str(diff_y) + "  theta:" + str(diff_theta)


            self.initposemsg.header.stamp = rospy.get_rostime()
            self.initposemsg.pose.pose.position.x = base_current_pos[0] + diff_x
            self.initposemsg.pose.pose.position.y = base_current_pos[1] + diff_y
            self.initposemsg.pose.pose.position.z = 0
            quaternion = tf.transformations.quaternion_from_euler(0, 0, base_current_theta + diff_theta)
            self.initposemsg.pose.pose.orientation.x = quaternion[0]
            self.initposemsg.pose.pose.orientation.y = quaternion[1]
            self.initposemsg.pose.pose.orientation.z = quaternion[2]
            self.initposemsg.pose.pose.orientation.w = quaternion[3]

            self.pub_initpose.publish(self.initposemsg)

        except Exception:
            None

    def update_shelf_pose_grid(self, scan_list, pos_list):
        shelf_pos_x = self.shelf_pos[0]
        shelf_pos_y = self.shelf_pos[1]
        shelf_theta = self.shelf_theta

        print "(x y theta) = (" + str(shelf_pos_x) + " " + str(shelf_pos_y) + " " + str(shelf_theta) + ")"

        prev_dist = self.transformed_scan_matching(
            scan_list, pos_list, shelf_pos_x, shelf_pos_y, shelf_theta)

        tx = 0
        ty = 0

        for y in numpy.arange(-0.1, 0.1, 0.025):
            for x in numpy.arange(-0.1, 0.1, 0.025):
                dist_o = self.transformed_scan_matching(
                    scan_list, pos_list,
                    shelf_pos_x + x, shelf_pos_y + y, shelf_theta)
                if prev_dist < dist_o:
                    prev_dist = dist_o
                    tx = x
                    ty = y

        return shelf_pos_x + tx, shelf_pos_y + ty, shelf_theta

    def update_shelf_pose_srv(self, req):
        x, y, theta = self.update_shelf_pose(self.scan_list, self.pos_list)
        #x, y, theta = self.update_shelf_pose_grid(self.mapgrid_list, self.pos_list)
        self.shelf_pos = (x, y, 0)
        self.shelf_quat = tf.transformations.quaternion_from_euler(
            0, 0, theta)
        return EmptyResponse()

    def update_init_pose_srv(self, req):
        x, y, theta = self.update_shelf_pose(self.scan_list, self.pos_list)
        #x, y, theta = self.update_shelf_pose_grid(self.mapgrid_list, self.pos_list)
        self.shelf_pos = (x, y, 0)
        self.shelf_quat = tf.transformations.quaternion_from_euler(
            0, 0, theta)
        self.update_init_pose(x, y, theta)
        return EmptyResponse()

    ## @brief create MarkerArray msg
    def vertices_to_marker_array(self):
        del self.markers.markers[:]
        marker = Marker()
        marker.header.frame_id = "new_" + self.shelf_base_link
        marker.type = Marker.LINE_STRIP
        marker.id = 0
        marker.action = 0
        marker.ns = 'shelf_box_scan_0'
        marker.pose.orientation.w = 1
        marker.scale = Vector3(x = 0.01, y = 0, z = 0)
        marker.color = ColorRGBA(r = 1, g = 0, b = 0, a = 1)
        for v in self.pos_list:
            point = Point(x = v[0], y = v[1], z = v[2])
            marker.points.append(point)
        self.markers.markers.append(marker)

    def publish_marker(self):
        for marker in self.markers.markers:
            marker.header.stamp = rospy.get_rostime()
        self.pub_marker.publish(self.markers)

    def publish_mapgrid(self):
        self.mapgrid_msg.header.stamp = rospy.get_rostime()
        self.pub_mapgrid.publish(self.mapgrid_msg)

    def publish_shelf_pose(self):
        self.tf_broadcaster.sendTransform(
            self.shelf_pos, self.shelf_quat, rospy.Time.now(),
            "new_" + self.shelf_base_link, "map")


if __name__ == '__main__':
    rospy.init_node('shelf_box_scan', anonymous = True)
    ## read from shelf_box_scan.yaml
    modelfile = (rospkg.RosPack().get_path('aero_maps') +
                 '/models/shelf_box_scan.yaml')
    model_fd = open(modelfile, 'r')
    model_dat = yaml.load(model_fd)
    model_fd.close()
    ## get shelf_base_link from param
    shelf_base_link = rospy.get_param('~shelf_base_link', 'shelf_base_link_0')

    box = ShelfBox(model_dat['x_pos_list'], model_dat['y_pos_list'], shelf_base_link)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        box.publish_marker()
        box.publish_mapgrid()
        box.publish_shelf_pose()
        rate.sleep()
