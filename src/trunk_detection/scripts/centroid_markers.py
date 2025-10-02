#!/usr/bin/env python3
# THIS SCRIPT TAKES THE POSES OF THE CLUSTER CENTROIDS AND PUBLISHES
# THEM AS MARKERS IN RVIZ

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray

class ClusterCentroidMarkers:

    def __init__ (self):
        rospy.init_node("cluster_centroid_markers", anonymous=True)


        self.sub_poses = rospy.Subscriber("/cluster_poses", PoseArray, self.pose_cb)
        self.pub_centroids = rospy.Publisher("/cluster_markers", MarkerArray, queue_size=1)


    def pose_cb(self, msg):
        m = Marker()
        m.header = msg.header                 # keep same frame+stamp as PoseArray
        m.ns = "cluster_centroids"
        m.id = 0
        m.type = Marker.POINTS                  # push all the poses into a POINTS
        m.action = Marker.ADD
        m.scale.x = 0.2                       # point width (m)
        m.scale.y = 0.2                       # point height (m)
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0

        for p in msg.poses:
            m.points.append(p.position)
        
        arr = MarkerArray()
        arr.markers.append(m)
        self.pub_centroids.publish(arr)



if __name__  == "__main__":
    ClusterCentroidMarkers()
    rospy.spin()