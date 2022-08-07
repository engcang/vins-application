#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import cv2
import sys
import signal

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError


def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class robot():
    def __init__(self):
	rospy.init_node('robot_controller', anonymous=True)
	#self.img_subscriber = rospy.Subscriber('/raspicam_node/image/compressed',CompressedImage,self.callback_compressed_img)
	self.img_publisher = rospy.Publisher('/image_mono',Image,queue_size=1)
	self.img_subscriber = rospy.Subscriber('/uav/camera/left/image_rect_color',Image,self.callback)
	self.imu_subscriber = rospy.Subscriber('/uav/sensors/imu', Imu, self.imu_callback)
	self.imu_publisher = rospy.Publisher('/imu_data', Imu, queue_size=1)
	self.bridge = CvBridge()

#    def callback_compressed_img(self,data):
#        np_arr = np.fromstring(data.data, np.uint8)
#	self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

    def callback(self,data):
	try :
#		self.time = time.time()
		cvimage=self.bridge.imgmsg_to_cv2(data,"bgr8")

		cv_image=cv2.cvtColor(cvimage,cv2.COLOR_BGR2GRAY)

		img=self.bridge.cv2_to_imgmsg(cv_image, "mono8")

		img.header.stamp = rospy.Time.now()
		self.img_publisher.publish(img)
#		print(time.time()-self.time)
	except CvBridgeError as e:
		pass

    def imu_callback(self,data):
	data.header.stamp = rospy.Time.now()
	self.imu_publisher.publish(data)

if __name__=='__main__':
	gray=robot()
	time.sleep(1)
	while 1:
		pass
