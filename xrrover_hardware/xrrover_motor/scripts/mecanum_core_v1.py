#!/usr/bin/env python

import rospy
import tf
from tf import transformations
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point
from nav_msgs.msg import Odometry
from math import sin, cos

class RosController:
    def __init__(self):
        # region: self variable init 
        self.aim_x = 0.0
        self.aim_y = 0.0
        self.aim_theta = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0

        self.real_fl_velocity = 0.0
        self.real_fr_velocity = 0.0
        self.real_rl_velocity = 0.0
        self.real_rr_velocity = 0.0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        # endregion

        # region: get parameters about the geometry of the wheels
        self.WHEEL_SEPARATION_WIDTH = rospy.get_param("/wheel/separation/horizontal")
        self.WHEEL_SEPARATION_LENGTH = rospy.get_param("/wheel/separation/vertical")
        self.WHEEL_RADIUS = rospy.get_param("/wheel/diameter") / 2
        self.WHEEL_GEOMETRY = (self.WHEEL_SEPARATION_WIDTH + self.WHEEL_SEPARATION_LENGTH) / 2
        # endregion

        # region: declare for cmd_vel -> four wheel aim speed
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.TriggerForMotorAimSpeedPublish)
        self.pub_mfl = rospy.Publisher('motor/front/left', Float32, queue_size=1)
        self.pub_mfr = rospy.Publisher('motor/front/right', Float32, queue_size=1)
        self.pub_mbl = rospy.Publisher('motor/rear/left', Float32, queue_size=1)
        self.pub_mbr = rospy.Publisher('motor/rear/right', Float32, queue_size=1)
        # endregion

        # region: declare for four wheel return speed -> odom and tf
        self.sub_wheel_fl = rospy.Subscriber('return/front/left', Float32, self.TriggerForUpdateFLVelocity)
        self.sub_wheel_fr = rospy.Subscriber('return/front/right', Float32, self.TriggerForUpdateFRVelocity)
        self.sub_wheel_rl = rospy.Subscriber('return/rear/left', Float32, self.TriggerForUpdateRLVelocity)
        self.sub_wheel_rr = rospy.Subscriber('return/rear/right', Float32, self.TriggerForUpdateRRVelocityThanPublishOdomAndTf)
        self.pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.odom_tf_broadcaster = tf.TransformBroadcaster()
        # endregion

    # region: implement cmd_vel -> four wheel aim speed
    def TriggerForMotorAimSpeedPublish(self, move):
        self.aim_x = move.linear.x
        self.aim_y = move.linear.y
        self.aim_theta = move.angular.z
        self.__PubFLMotorAimSpeed()
        self.__PubFRMotorAimSpeed()
        self.__PubRLMotorAimSpeed()
        self.__PubRRMotorAimSpeed()

    def __PubFLMotorAimSpeed(self):
        motor_speed = (self.aim_x - self.aim_y - self.aim_theta * self.WHEEL_GEOMETRY) / self.WHEEL_RADIUS
        self.pub_mfl.publish(motor_speed)

    def __PubFRMotorAimSpeed(self):
        motor_speed = (self.aim_x + self.aim_y + self.aim_theta * self.WHEEL_GEOMETRY) / self.WHEEL_RADIUS
        self.pub_mfr.publish(motor_speed)

    def __PubRLMotorAimSpeed(self):
        motor_speed = (self.aim_x + self.aim_y - self.aim_theta * self.WHEEL_GEOMETRY) / self.WHEEL_RADIUS
        self.pub_mbl.publish(motor_speed)

    def __PubRRMotorAimSpeed(self):
        motor_speed = (self.aim_x - self.aim_y + self.aim_theta * self.WHEEL_GEOMETRY) / self.WHEEL_RADIUS
        self.pub_mbr.publish(motor_speed)
    # endregion

    # region: implement four wheel return speed -> odom and tf
    def TriggerForUpdateFLVelocity(self, single_wheel):
        self.real_fl_velocity = -single_wheel.data

    def TriggerForUpdateFRVelocity(self, single_wheel):
        self.real_fr_velocity = single_wheel.data

    def TriggerForUpdateRLVelocity(self, single_wheel):
        self.real_rl_velocity = -single_wheel.data

    def TriggerForUpdateRRVelocityThanPublishOdomAndTf(self, single_wheel):
        self.real_rr_velocity = single_wheel.data
        self.__publishOdomAndTf()

    def __deltaTimeUpdateTrajectory(self):
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        delta_x = (self.odom_vx * cos(self.odom_theta) - self.odom_vy * sin(self.odom_theta)) * dt
        delta_y = (self.odom_vx * sin(self.odom_theta) + self.odom_vy * cos(self.odom_theta)) * dt
        delta_th = self.odom_wz * dt

        self.odom_x += delta_x
        self.odom_y += delta_y
        self.odom_theta += delta_th
        self.odom_quaternion = transformations.quaternion_from_euler(0, 0, self.odom_theta)

        self.last_time = self.current_time

    def __UpdateOdomTwist(self):
        # forward kenimatic
        self.odom_vx = ( self.real_fl_velocity + self.real_fr_velocity + self.real_rl_velocity + self.real_rr_velocity) / 4
        self.odom_vy = (-self.real_fl_velocity + self.real_fr_velocity + self.real_rl_velocity - self.real_rr_velocity) / 4
        self.odom_wz = (-self.real_fl_velocity + self.real_fr_velocity - self.real_rl_velocity + self.real_rr_velocity) / (4 * self.WHEEL_GEOMETRY) 

    def __publishOdomAndTf(self): 

        self.__deltaTimeUpdateTrajectory()
        self.__UpdateOdomTwist()

        # region: pub tf
        self.odom_tf_broadcaster.sendTransform(
            (self.odom_x, self.odom_y, 0.),
            self.odom_quaternion,
            self.current_time,
            "base_footprint",
            "odom"
        )
        # endregion

        # region: pub odom
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(self.odom_vx, self.odom_vy, 0), Vector3(0, 0, self.odom_wz))
        odom.pose.pose = Pose(Point(self.odom_x, self.odom_y, 0.), Quaternion(*self.odom_quaternion))
        self.pub.publish(odom)
        # endregion
        pass
    #endregion

if __name__ == '__main__':
    try:
        rospy.init_node('xrrover_controller')
        mecanum_core = RosController()
        
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
