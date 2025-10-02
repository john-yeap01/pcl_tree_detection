#!/usr/bin/env python
import rospy
import math
import tf.transformations as transformations
from geographiclib.geodesic import Geodesic
from geometry_msgs.msg import PoseStamped

class TransformUtils:

    def __init__(self, origin_lat, origin_lon, origin_x, origin_y):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_x = origin_x
        self.origin_y = origin_y
        self._geod = Geodesic.WGS84

    def update_origin(self, lat, lon, x, y):
        self.origin_lat = lat
        self.origin_lon = lon
        self.origin_x = x 
        self.origin_y = y


    # convert GPS coordinate into a local Cartesian relative to stored origin
    def gps_to_local(self, lat, long):
        g = self._geod.Inverse(
            self.origin_lat, self.origin_lon,
            lat, long
        )
        dist = g['s12']
        ang  = math.radians(g['azi1'])

        # local XY offsets
        dx = dist * math.sin(ang)
        dy = dist * math.cos(ang)

        local_x = self.origin_x + dx
        local_y = self.origin_y + dy

        return local_x, local_y

    def local_to_gps(self, x, y):
        dx = x - self.origin_x
        dy = y - self.origin_y

        dist = math.hypot(dx, dy)
        azimuth_rad = math.atan2(dx, dy)
        azimuth_deg = math.degrees(azimuth_rad) % 360

        pt = self._geod.Direct(
            self.origin_lat, self.origin_lon,
            azimuth_deg, dist
        )

        lat2 = pt['lat2']
        long2 = pt['lon2']

        return lat2, long2

    
    def make_pose(self, lat, long, alt, yaw_rad, frame_id = "world"):
        local_x, local_y = self.gps_to_local(lat, long)

        quat = self.yaw_to_quat(yaw_rad)

        pose = PoseStamped()
        pose.header.frame_id = frame_id             # use world frame (not "map")
        pose.pose.position.x = local_x
        pose.pose.position.y = local_y
        pose.pose.position.z = alt                  # use absolute altitude from mission planner
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def quat_to_yaw(self, q):
        _, _, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw
    
    def yaw_to_quat(self, yaw):
        return transformations.quaternion_from_euler(0, 0, yaw)

    def rad_to_deg(yaw_rad):
        return (math.degrees(yaw_rad) + 360) % 360
    
    def quat_xyzw_to_yaw(self, qx, qy, qz, qw):
        _, _, yaw = transformations.euler_from_quaternion([qx, qy, qz, qw])
        return yaw
