#!/usr/bin/env python

# region: import module
import rospy
import tf
from tf import transformations
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, Point
from nav_msgs.msg import Odometry
from math import sin, cos
# endregion

# region: declare global variable
rospy.init_node('mecanum')

current_time = rospy.Time.now()
last_time = rospy.Time.now()

x, y, theta = 0.0, 0.0, 0.0
vx, vy, wz = 0.0, 0.0, 0.0
vel_fl, vel_fr, vel_rl, vel_rr = 0.0, 0.0, 0.0, 0.0
# endregion

def inverseKinematicWithPublisher(move):
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

def forwardKinematic(fl, fr, rl, rr):
    vx = (-fl + fr - rl + rr)/4
    vy = (fl + fr - rl - rr)/4
    wz = (fl + fr + rl + rr)/(4*WHEEL_GEOMETRY)
    rospy.loginfo(vy)
    return vx, vy, wz

# region callback funciton for each wheel velocity
def flCallback(single_wheel):
    global vel_fl
    vel_fl = single_wheel.data

def frCallback(single_wheel):
    global vel_fr
    vel_fr = single_wheel.data

def rlCallback(single_wheel):
    global vel_rl
    vel_rl = single_wheel.data

def rrCallback(single_wheel):
    global vel_rr
    vel_rr = single_wheel.data
    publishOdomAndTf()
# endregion

def publishOdomAndTf():
    global current_time, last_time, vx, vy, wz, theta, x, y, vel_fl, vel_fr, vel_rl, vel_rr
    
    # region: compute x, y, theta for odom and tf
    current_time = rospy.Time.now()
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(theta) - vy * sin(theta)) * dt
    delta_y = (vx * sin(theta) + vy * cos(theta)) * dt
    delta_th = wz * dt
    x += delta_x
    y += delta_y
    theta += delta_th
    # endregion

    odom_quaternion = transformations.quaternion_from_euler(0, 0, theta)

    # region: pub tf
    odom_tf_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quaternion,
        current_time,
        "base_link",
        "odom"
    )
    # endregion

    # compute twist
    vx, vy, wz = forwardKinematic(vel_fl, vel_fr, vel_rl, vel_rr)

    # region: pub odom
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, wz))
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quaternion))
    pub.publish(odom)
    # endregion
    
    last_time = current_time

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            # region: get parameters about the geometry of the wheels
            WHEEL_SEPARATION_WIDTH = rospy.get_param("/wheel/separation/horizontal")
            WHEEL_SEPARATION_LENGTH = rospy.get_param("/wheel/separation/vertical")
            WHEEL_GEOMETRY = (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) / 2
            WHEEL_RADIUS = rospy.get_param("/wheel/diameter") / 2
            # endregion

            # region: publisher
            pub = rospy.Publisher('odom', Odometry, queue_size=1)
            odom_tf_broadcaster = tf.TransformBroadcaster()
            pub_mfl = rospy.Publisher('motor/front/left', Float32, queue_size=1)
            pub_mfr = rospy.Publisher('motor/front/right', Float32, queue_size=1)
            pub_mbl = rospy.Publisher('motor/rear/left', Float32, queue_size=1)
            pub_mbr = rospy.Publisher('motor/rear/right', Float32, queue_size=1)
            # endregion

            # region: subscriber
            sub = rospy.Subscriber('cmd_vel', Twist, inverseKinematicWithPublisher)
            sub_wheel_fl = rospy.Subscriber('return/front/left', Float32, flCallback)
            sub_wheel_fr = rospy.Subscriber('return/front/right', Float32, frCallback)
            sub_wheel_rl = rospy.Subscriber('return/rear/left', Float32, rlCallback)
            sub_wheel_rr = rospy.Subscriber('return/rear/right', Float32, rrCallback)
            # endregion

            rospy.spin()

    except rospy.ROSInterruptException:
        pass
