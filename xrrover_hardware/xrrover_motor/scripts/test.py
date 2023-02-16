#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import Float32

class mycontroller:
    def __init__(self):
        self.pub = rospy.Publisher('jerrynum', Float32, queue_size=1)
        self.pub2 = rospy.Publisher('jerrynum2', Float32, queue_size=1)
        self.rate1 = rospy.Rate(10)

    def init_pubMsg(self):
        jerrynum = Float32()
        jerrynum.data = 32.00
        
        self.pub.publish(jerrynum)

    def init_pub2Msg(self):
        jerrynum = Float32()
        jerrynum.data = 5.00
        
        self.pub2.publish(jerrynum)

    def rateforsleep(self):
        self.rate1.sleep()


if __name__ == "__main__":
    rospy.init_node('test')
    jerryros = mycontroller()


    while not rospy.is_shutdown():

        jerryros.init_pubMsg()
        jerryros.init_pub2Msg()

        jerryros.rateforsleep()