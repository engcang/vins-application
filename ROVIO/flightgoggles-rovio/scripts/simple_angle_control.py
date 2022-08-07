#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019

@author: mason
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion
import numpy as np

import rospy
from mav_msgs.msg import RateThrust
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


global joy_check
joy_check=0

global d2r
global r2d
global g
global max_ang_x
global max_ang_y

g = 9.81
r2d = 180/np.pi
d2r = np.pi/180
max_ang_x = 40 * d2r # max angle -> edit here to change velocity of vehicle
max_ang_y = 40 * d2r

''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.local_deg_pub = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=10)
        self.reset_pub = rospy.Publisher('/uav/collision', Empty, queue_size=1)
        self.tf_pos_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.joy_sub = rospy.Subscriber('/control_nodes/joy', Joy, self.joy_callback)

        self.rate = rospy.Rate(30)

        self.pose = TFMessage()
        self.deg = RateThrust()
        self.joy = Joy()
        self.reset = Empty()
        self.mode = 2 #default is mode 2

    def tf_callback(self, msg):
#        for i in range(0,len(msg.transforms)):
	i=len(msg.transforms)-1
        if msg.transforms[i].child_frame_id=="uav/imu":
            self.truth=msg.transforms[i].transform.translation
            orientation_list = [msg.transforms[i].transform.rotation.x, msg.transforms[i].transform.rotation.y, msg.transforms[i].transform.rotation.z, msg.transforms[i].transform.rotation.w]
            (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def joy_callback(self, msg):
        self.joy = msg
        global joy_check
        if len(self.joy.axes)>0 or len(self.joy.buttons)>0 :
            joy_check=1
            if self.joy.buttons[4]==1:
                self.mode=1
            if self.joy.buttons[5]==1:
                self.mode=2
            if self.joy.buttons[0]==1:
                self.reset_pub.publish(self.reset)

def input(rbt):
    global d2r
    global r2d
    global max_ang_x
    global max_ang_y
    global g

##Mode 2, default
#joy_axes: {pitch: 4, roll: 3, yaw: 0, vertical: 1}
    if rbt.mode==2:
        rbt.deg.angular_rates.x=2*(-rbt.joy.axes[3]*max_ang_x - rbt.roll)
        rbt.deg.angular_rates.y=2*( rbt.joy.axes[4]*max_ang_y - rbt.pitch)
        rbt.deg.angular_rates.z=1.5*(rbt.joy.axes[0])
        rbt.deg.thrust.z = g + rbt.joy.axes[1]*g #throttle
##Mode 1
#joy_axes: {pitch: 1, roll: 3, yaw: 0, vertical: 4}
    elif rbt.mode==1:
        rbt.deg.angular_rates.x=2*(-rbt.joy.axes[3]*max_ang_x - rbt.roll)
        rbt.deg.angular_rates.y=2*( rbt.joy.axes[1]*max_ang_y - rbt.pitch)
        rbt.deg.angular_rates.z=1.5*(rbt.joy.axes[0])
        rbt.deg.thrust.z = g + rbt.joy.axes[4]*g #throttle

### removing quivering ###
    if abs(rbt.deg.angular_rates.x) < 0.08:
        rbt.deg.angular_rates.x=0
    if abs(rbt.deg.angular_rates.y) < 0.08:
        rbt.deg.angular_rates.y=0
    if abs(rbt.deg.angular_rates.z) < 0.08:
        rbt.deg.angular_rates.z=0    

    print("Mode < %d > now, press L1 or R1 to change, Press Button[0] to reset Simulator"%rbt.mode)
    print("Input : X: %.3f  Y: %.3f  Z: %.3f  Throttle: %.3f"%(rbt.deg.angular_rates.x, rbt.deg.angular_rates.y, rbt.deg.angular_rates.z, rbt.deg.thrust.z))
    print("Angle(Degree): roll: %.4f pitch: %.4f yaw: %.4f \n"%(rbt.roll/np.pi*180, rbt.pitch/np.pi*180, rbt.yaw/np.pi*180)) #radian : +-pi

    rbt.deg.header.stamp = rospy.Time.now()
    rbt.local_deg_pub.publish(rbt.deg)

##############################################################################################

alpha = robot()
alpha.joy_callback(alpha.joy)
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            if joy_check==1:
                input(alpha)
                alpha.rate.sleep()
            else: 
                alpha.rate.sleep()
                pass
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass
