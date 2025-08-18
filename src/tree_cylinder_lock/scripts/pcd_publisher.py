#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg