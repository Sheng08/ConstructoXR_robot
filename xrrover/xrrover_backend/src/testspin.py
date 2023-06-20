#!/usr/bin/env python

import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseBackend():

    def __init__(self):

        rospy.init_node('xrrover_movebase_backend')
        
        
        # Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz')))
        # Pose(Point(*point),quat_seq[n-3])

        self.create_action_client()        
        self.connect_movebase_server()
        self.post_navigation_goal()

    def create_action_client(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def connect_movebase_server(self):
        response = self.client.wait_for_server(rospy.Duration(secs=5.0))
        if not response:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")

    def post_navigation_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        pose = Pose(Point(0.5, 0.5, 0), Quaternion(*quaternion_from_euler(0, 0, 0)))
        goal.target_pose.pose = pose
        rospy.loginfo(f"Sending goal pose {str(self.pose)} to Action Server")
        self.client.send_goal(goal, self.done_callback, self.active_callback, self.feedback_callback)
        rospy.spin()

    def active_callback(self):
        rospy.loginfo("[ACTIVE]: Goal pose is now being processed by the Action Server...")

    def feedback_callback(self, feedback):
        rospy.loginfo(f"[FEEDBACK]: now pose :\n{str(feedback)}.")

    def done_callback(self, status, result):
        if status == 0:
            rospy.loginfo("[PENDING]: Goal pose has yet to be processed.")

        if status == 2:
            rospy.loginfo("[PREEMPTED]: Goal pose received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("[SUCCEEDED]: Goal pose reached.")
            return

        if status == 4:
            rospy.loginfo("[ABORT]: Goal pose was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("[REJECTED]: Goal pose has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("[RECALLED]: Goal pose received a cancel request before it started executing, successfully cancelled!")


if __name__ == '__main__':
    try:
        MoveBaseBackend()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")