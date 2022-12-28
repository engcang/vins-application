#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue May 19 00:28:30 2020

@author: mason
"""

''' import libraries '''
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Imu

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''

class visualizer():
    def __init__(self):
        rospy.init_node('path_pubb', anonymous=True)
        self.imu_pub = rospy.Publisher("/imu0", Imu, queue_size=1)
        self.img_pub = rospy.Publisher("/cam0/image_raw", Image, queue_size=1)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        self.img_sub = rospy.Subscriber("/camera/infra1/image_rect_raw", Image, self.img_cb)
        
        self.gt_sub = rospy.Subscriber("/pose_transformed", PoseStamped, self.gt_cb)
        self.gt_path_pub = rospy.Publisher("/gt_path", Path, queue_size=1)
        self.dmvio_sub = rospy.Subscriber("/dmvio/metric_pose", PoseStamped, self.pose_cb)
        self.dmvio_path_pub = rospy.Publisher("/dmvio_path", Path, queue_size=1)

        self.rate = rospy.Rate(1)

        self.gt_path = Path()
        self.gt_path.header.frame_id = "world"
        self.gt_counter=0
        self.dmvio_path = Path()
        self.dmvio_path.header.frame_id = "world"
        self.dmvio_counter=0

    def imu_cb(self, msg):
        self.imu_pub.publish(msg)
    def img_cb(self, msg):
        self.img_pub.publish(msg)

    def gt_cb(self, msg):
        self.gt_counter = self.gt_counter+1
        if self.gt_counter % 10 == 0:
            self.gt_path.poses.append(msg)
            self.gt_path_pub.publish(self.gt_path)

    def pose_cb(self, msg):
        self.dmvio_counter = self.dmvio_counter+1
        if self.dmvio_counter % 2 == 0:
            self.dmvio_path.poses.append(msg)
            self.dmvio_path_pub.publish(self.dmvio_path)

''' main '''
vis_ = visualizer()

if __name__ == '__main__':
    while 1:
        try:
            vis_.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        # except:
        #     print("exception")
        #     pass
