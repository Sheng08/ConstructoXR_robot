#!/usr/bin/env python

from email import message
import rospy
import tf
from tf import transformations
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point
from nav_msgs.msg import Odometry
from math import sin, cos
import message_filters

rospy.init_node('mecanum')

WHEEL_SEPARATION_WIDTH = None
WHEEL_SEPARATION_LENGTH = None
WHEEL_GEOMETRY = None
WHEEL_RADIUS = None

pub_mfl = None
pub_mfr = None
pub_mbl = None
pub_mbr = None


# class VelFromWheel:
#     vel_fl = 0.0
#     vel_fr = 0.0
#     vel_rl = 0.0
#     vel_rr = 0.0

# receive = VelFromWheel()

def convert(move):
    x = move.linear.x
    y = move.linear.y
    rot = move.angular.z

    front_left = (x - y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    front_right = (x + y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_left = (x + y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_right = (x - y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS

    pub_mfl.publish(front_left)
    pub_mfr.publish(front_right)
    pub_mbl.publish(back_left)
    pub_mbr.publish(back_right)

# def flCallback(single_wheel):
#     receive.vel_fl = single_wheel.data

# def frCallback(single_wheel):
#     receive.vel_fr = single_wheel.data

# def rlCallback(single_wheel):
#     receive.vel_rl = single_wheel.data

# def rrCallback(single_wheel):
#     receive.vel_rr = single_wheel.data
#     pub.publish(odom)
#     rospy.loginfo("where")

# def computeOdom_twist():

#     return vx, vy, wz

# def computeOdom_pose():
#     current_time = rospy.Time.now()

#     # compute odometry in a typical way given the velocities of the robot
#     dt = (current_time - last_time).to_sec()
#     delta_x = (vx * cos(th) - vy * sin(th)) * dt
#     delta_y = (vx * sin(th) + vy * cos(th)) * dt
#     delta_th = wz * dt
    
#     x += delta_x
#     y += delta_y
#     th += delta_th

#     odom_quat = transformations.quaternion_from_euler(0, 0, th)

#     odom_broadcaster.sendTransform(
#         (x, y, 0.),
#         odom_quat,
#         current_time,
#         "base_link",
#         "odom"
#     )

#     odom = Odometry()
#     odom.header.stamp = current_time
#     odom.header.frame_id = "odom"

#     odom.child_frame_id = "base_link"

#     vx, vy, wz = computeOdom_twist()
#     odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, wz))

#     odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
#     pub.publish(odom)

def odomCallback(fl, fr, rl, rr):
    current_time = rospy.Time.now()
    vx = (fl.data + fr.data + rl.data + rr.data)/4
    vy = (-fl.data + fr.data + rl.data - rr.data)/4
    wz = (-fl.data + fr.data - rl.data + rr.data)/(4*WHEEL_GEOMETRY)

    current_time = rospy.Time.now()
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = wz * dt

    x += delta_x
    y += delta_y
    th += delta_th

    odom_quat = transformations.quaternion_from_euler(0, 0, th)

    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.child_frame_id = "base_link"

    vx = (fl.data + fr.data + rl.data + rr.data)/4
    vy = (-fl.data + fr.data + rl.data - rr.data)/4
    wz = (-fl.data + fr.data - rl.data + rr.data)/(4*WHEEL_GEOMETRY)
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, wz))

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    last_time = current_time
    pub.publish(odom)

if __name__ == '__main__':
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    th = 0.0
    try:
        while not rospy.is_shutdown():
            

            # Get parameters about the geometry of the wheels
            WHEEL_SEPARATION_WIDTH = rospy.get_param("/wheel/separation/horizontal")
            WHEEL_SEPARATION_LENGTH = rospy.get_param("/wheel/separation/vertical")
            WHEEL_GEOMETRY = (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) / 2
            WHEEL_RADIUS = rospy.get_param("/wheel/diameter") / 2

            pub = rospy.Publisher('odom', Odometry, queue_size=1)
            odom_broadcaster = tf.TransformBroadcaster()
            pub_mfl = rospy.Publisher('motor/front/left', Float32, queue_size=1)
            pub_mfr = rospy.Publisher('motor/front/right', Float32, queue_size=1)
            pub_mbl = rospy.Publisher('motor/rear/left', Float32, queue_size=1)
            pub_mbr = rospy.Publisher('motor/rear/right', Float32, queue_size=1)

            sub = rospy.Subscriber('cmd_vel', Twist, convert)

            sub_wheel_fl = message_filters.Subscriber('return/front/left', Float32)
            sub_wheel_fr = message_filters.Subscriber('return/front/right', Float32)
            sub_wheel_rl = message_filters.Subscriber('return/rear/left', Float32)
            sub_wheel_rr = message_filters.Subscriber('return/rear/right', Float32)
            ts = message_filters.TimeSynchronizer([sub_wheel_fl, sub_wheel_fr, sub_wheel_rl, sub_wheel_rr],10)

            x = 0.0
            y = 0.0
            
            vx = 0
            vy = 0
            wz = 0
            ts.registerCallback(odomCallback)

            rospy.loginfo("here")
            
            rospy.spin()

    except rospy.ROSInterruptException:
        pass
