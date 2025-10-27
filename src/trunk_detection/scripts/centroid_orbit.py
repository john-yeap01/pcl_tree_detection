#!/usr/bin/env python3
# Subscribes to clusters and then plots the orbit points and executes them
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
import math

# --- Tunables (kept small & clear) ---
PUBLISH_RATE_HZ  = 5.0    # keep re-publishing current goal for controllers that need continuous setpoints
PAUSE_DURATION   = 1.0    # small pause after arrival (seconds)
WAIT_START_TO    = 15.0   # timeout to observe FOLLOW_TRAJ after publishing a goal (seconds)
WAIT_ARRIVE_TO   = 90.0   # timeout to observe WAIT_GOAL/PAUSING after movement (seconds)
ONLY_FIRST_POSE  = True   # if True, use only the first pose from /cluster_poses

def yaw_to_quat(yaw):
    cy = math.cos(0.5*yaw); sy = math.sin(0.5*yaw)
    q = Quaternion(); q.x = 0.0; q.y = 0.0; q.z = sy; q.w = cy
    return q


class ClusterPoseNavigator:
    def __init__(self):
        rospy.init_node("cluster_pose_navigator", anonymous=True)

        # ---- State ----
        self.waypoints = []              # list of PoseStamped (in source frame)
        self.current_goal_index = 0
        self.last_sent_goal = None

        self.last_fsm_state = None       # e.g., INIT, FOLLOW_TRAJ, WAIT_GOAL, PAUSING, ...
        self.current_mavros_mode = None  # GUIDED / LOITER etc.
        self.prev_mavros_mode = None
        self.have_plan = False

        # ---- Pub/Sub ----
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
        rospy.Subscriber("/cluster_poses", PoseArray, self.cluster_cb, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self.mavros_state_cb, queue_size=10)
        rospy.Subscriber("/super_fsm_state", String, self.fsm_state_cb, queue_size=20)

        rospy.loginfo("cluster_pose_navigator up. Waiting for /cluster_poses…")
        rospy.spin()

    # ----------------- Callbacks -----------------
    def cluster_cb(self, msg: PoseArray):
        if self.have_plan:
            return
        if not msg.poses:
            return

        frame = msg.header.frame_id if msg.header.frame_id else "map"
        center = msg.poses[0].position
        cx, cy, cz = center.x, center.y, msg.poses[0].position.z

        R = rospy.get_param("~orbit_radius", 1.5)      # radius (m)
        angles_deg = [0, 90, 180, 270]                  # 4 goals
        now = rospy.Time.now()

        self.waypoints = []
        for deg in angles_deg:
            th = math.radians(deg)
            x = cx + R * math.cos(th)
            y = cy + R * math.sin(th)
            z = cz                                     # keep cluster altitude

            # yaw faces the center
            yaw = math.atan2(cy - y, cx - x)

            g = PoseStamped()
            g.header.frame_id = frame
            g.header.stamp = now
            g.pose.position.x = x
            g.pose.position.y = y
            g.pose.position.z = z
            g.pose.orientation = yaw_to_quat(yaw)
            self.waypoints.append(g)

        self.have_plan = True
        self.current_goal_index = 0
        rospy.loginfo("Planned %d orbit goals around cluster at (%.2f, %.2f, %.2f).",
                    len(self.waypoints), cx, cy, cz)

        # Kick off immediately if idle-like
        if self.last_fsm_state in (None, "INIT", "WAIT_GOAL", "IDLE", "PAUSING"):
            self.advance_through_goals()


    def mavros_state_cb(self, msg: State):
        self.prev_mavros_mode = self.current_mavros_mode
        self.current_mavros_mode = msg.mode

        # GUIDED -> LOITER: remember paused goal
        if self.prev_mavros_mode == "GUIDED" and msg.mode == "LOITER":
            rospy.loginfo("Mode LOITER: pausing on goal #%d", max(self.current_goal_index - 1, 0))
            # nothing else to do; we keep last_sent_goal around

        # LOITER -> GUIDED: re-publish paused goal if we had one
        if self.prev_mavros_mode == "LOITER" and msg.mode == "GUIDED" and self.last_sent_goal:
            self.last_sent_goal.header.stamp = rospy.Time.now()
            self.goal_pub.publish(self.last_sent_goal)
            rospy.loginfo("Mode GUIDED: re-publishing paused goal")

    def fsm_state_cb(self, msg: String):
        new_state = (msg.data or "").strip().upper()
        if new_state == self.last_fsm_state:
            return
        rospy.loginfo("[FSM] %s -> %s", self.last_fsm_state, new_state)
        self.last_fsm_state = new_state
        # Progression is handled inside advance_through_goals()

    # -------------- Main goal progression --------------
    def advance_through_goals(self):
        """Publish each goal, wait for FSM transitions (FOLLOW_TRAJ then WAIT_GOAL/PAUSING), then proceed."""
        if not self.have_plan or not self.waypoints:
            rospy.logwarn("No goals planned yet.")
            return

        rate = rospy.Rate(PUBLISH_RATE_HZ)

        while not rospy.is_shutdown() and self.current_goal_index < len(self.waypoints):
            goal = self.waypoints[self.current_goal_index]
            rospy.loginfo("→ Goal #%d / %d (x=%.2f, y=%.2f, z=%.2f)",
                          self.current_goal_index + 1, len(self.waypoints),
                          goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)

            # 1) Publish until we see FOLLOW_TRAJ (or timeout)
            t0 = rospy.Time.now()
            while not rospy.is_shutdown():
                goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(goal)
                self.last_sent_goal = goal

                if self.last_fsm_state in ("FOLLOW_TRAJ", "FOLLOW_TRAJECTORY", "MOVING"):
                    rospy.loginfo("Movement started (FSM=%s).", self.last_fsm_state)
                    break

                if (rospy.Time.now() - t0).to_sec() > WAIT_START_TO:
                    rospy.logwarn("No FOLLOW_TRAJ observed within %.1fs — continuing.", WAIT_START_TO)
                    break

                rate.sleep()

            # 2) Keep publishing until we see WAIT_GOAL / PAUSING / IDLE (arrival) or timeout
            t1 = rospy.Time.now()
            arrived = False
            while not rospy.is_shutdown():
                goal.header.stamp = rospy.Time.now()
                self.goal_pub.publish(goal)
                self.last_sent_goal = goal

                if self.last_fsm_state in ("WAIT_GOAL", "PAUSING", "IDLE"):
                    rospy.loginfo("Arrival detected (FSM=%s).", self.last_fsm_state)
                    arrived = True
                    break

                if (rospy.Time.now() - t1).to_sec() > WAIT_ARRIVE_TO:
                    rospy.logwarn("Arrival timeout after %.1fs — proceeding to next.", WAIT_ARRIVE_TO)
                    break

                rate.sleep()

            if arrived:
                rospy.sleep(PAUSE_DURATION)

            self.current_goal_index += 1

        rospy.loginfo("All cluster goals processed — shutting down.")
        rospy.signal_shutdown("Mission complete")

if __name__ == "__main__":
    try:
        ClusterPoseNavigator()
    except rospy.ROSInterruptException:
        pass
