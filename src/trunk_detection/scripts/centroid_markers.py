#!/usr/bin/env python3
# THIS SCRIPT TAKES CYLINDER COEFFICIENTS AND/OR POSE ARRAYS
# AND PUBLISHES MARKERS IN RVIZ

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray
from pcl_msgs.msg import ModelCoefficients  # <<< NEW
import math

class ClusterCentroidMarkers:
    def __init__ (self):
        rospy.init_node("cluster_centroid_markers", anonymous=True)
        self.sub_poses = rospy.Subscriber("/cluster_poses", PoseArray, self.cluster_cb)
        self.sub_coef  = rospy.Subscriber("/cylinder_coeffs", ModelCoefficients, self.coeff_cb)  
        self.sub_cyl_centroid = rospy.Subscriber("/cylinder_centroids", PoseArray, self.centroid_cb)
        self.pub_markers = rospy.Publisher("/cluster_markers", MarkerArray, queue_size=1)

    def cluster_cb(self, msg):
        m = Marker()
        m.header = msg.header
        m.ns = "cluster_centroids"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0
        for p in msg.poses:
            m.points.append(p.position)
            rospy.loginfo("centroid at (%.2f, %.2f, %.2f)", p.position.x, p.position.y, p.position.z)
        arr = MarkerArray(); arr.markers.append(m)
        # self.pub_markers.publish(arr)

    def coeff_cb(self, mc):
        vals = mc.values
        if len(vals) < 7:
            rospy.logwarn("cylinder coeffs malformed")
            return
        px, py, pz, ax, ay, az, r = vals[:7]

        # Minimal marker: align with Z (no quaternion math), fixed height 1.0 m
        m = Marker()
        m.header = mc.header
        m.ns = "cylinder_fit"
        m.id = 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = px
        m.pose.position.y = py
        m.pose.position.z = pz
        m.pose.orientation.w = 1.0  # identity; axis = Z
        m.scale.x = 2.0 * r
        m.scale.y = 2.0 * r
        m.scale.z = 100.0             # minimal; replace with true height later
        m.color.r = 0.2
        m.color.g = 0.8
        m.color.b = 0.3
        m.color.a = 0.8

        arr = MarkerArray(); arr.markers.append(m)
        # self.pub_markers.publish(arr)
        rospy.loginfo("cylinder: center=(%.2f,%.2f,%.2f) r=%.3f", px,py,pz,r)


    def centroid_cb(self, msg):
        if not msg.poses:
            return
        p = msg.poses[0].position

        m = Marker()
        m.header = msg.header
        m.ns = "cylinder_centroid"
        m.id = 0
        m.type = Marker.SPHERE          # a clear dot at the centroid
        m.action = Marker.ADD
        m.pose.position = p
        m.pose.orientation.w = 1.0      # identity
        m.scale.x = 0.5                 # 15 cm ball; adjust as you like
        m.scale.y = 0.5
        m.scale.z = 0.5
        m.color.r = 1.0
        m.color.g = 0.9
        m.color.b = 0.1
        m.color.a = 1.0

        arr = MarkerArray()
        arr.markers.append(m)
        self.pub_markers.publish(arr)

        

if __name__  == "__main__":
    ClusterCentroidMarkers()
    rospy.spin()
